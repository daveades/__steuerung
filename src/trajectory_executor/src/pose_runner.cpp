#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include "arm_kinematics.hpp"
#include <dofbot_description/grasp_kinematics.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

class PoseRunner : public rclcpp::Node
{
public:
    PoseRunner()
        : Node("pose_runner")
    {
        rcl_interfaces::msg::ParameterDescriptor startupDescriptor;
        startupDescriptor.read_only = true;
        continuousMotion = declare_parameter<bool>("continuous_motion", false, startupDescriptor);
        poseName = declare_parameter<std::string>("pose_name", "home", startupDescriptor);
        targetXyz = declare_parameter<std::vector<double>>("target_xyz", {}, startupDescriptor);
        pickXyz = declare_parameter<std::vector<double>>("pick_xyz", {}, startupDescriptor);
        placeXyz = declare_parameter<std::vector<double>>("place_xyz", {}, startupDescriptor);
        pickPlaceMode = !pickXyz.empty() || !placeXyz.empty();
        const bool xyzMode = !targetXyz.empty();
        targetFrame = declare_parameter<std::string>("target_frame", "arm_link5", startupDescriptor);
        const auto approachMode = declare_parameter<std::string>("approach_mode", "position_only", startupDescriptor);
        if (approachMode != "position_only" && approachMode != "downward" && approachMode != "fixed_direction")
        {
            throw std::invalid_argument("approach_mode must be position_only, downward or fixed_direction");
        }
        downwardApproach = approachMode == "downward";
        if (downwardApproach && targetFrame != "grasp_contact_frame")
        {
            throw std::invalid_argument("Downward approach requires target_frame:=grasp_contact_frame");
        }
        const auto requestedDirection = declare_parameter<std::vector<double>>("approach_direction", {}, startupDescriptor);
        if (approachMode == "fixed_direction")
        {
            if (targetFrame != "grasp_contact_frame")
            {
                throw std::invalid_argument("Fixed-direction approach requires target_frame:=grasp_contact_frame");
            }
            if (!valid_xyz(requestedDirection))
            {
                throw std::invalid_argument("approach_direction must contain three finite coordinates relative to base_link");
            }
            const double length = std::hypot(requestedDirection[0], requestedDirection[1], requestedDirection[2]);
            if (!std::isfinite(length) || length < 1e-12)
            {
                throw std::invalid_argument("approach_direction must have finite nonzero length");
            }
            approachDirection = std::array<double, 3>{requestedDirection[0] / length,
                requestedDirection[1] / length, requestedDirection[2] / length};
        }
        else if (!requestedDirection.empty())
        {
            throw std::invalid_argument("approach_direction requires approach_mode:=fixed_direction");
        }
        if (targetFrame != "arm_link5" && targetFrame != "grasp_contact_frame")
        {
            throw std::invalid_argument("target_frame must be arm_link5 or grasp_contact_frame");
        }
        if (targetFrame == "grasp_contact_frame" && (!xyzMode || pickPlaceMode))
        {
            throw std::invalid_argument("grasp_contact_frame requires a single target_xyz request");
        }
        const auto &parameterOverrides = get_node_parameters_interface()->get_parameter_overrides();
        if (targetFrame == "grasp_contact_frame" &&
            (parameterOverrides.count("pose_name") != 0 ||
             std::any_of(parameterOverrides.begin(), parameterOverrides.end(),
                [](const auto &parameter) { return parameter.first.starts_with("poses."); })))
        {
            throw std::invalid_argument("Contact XYZ targeting cannot be combined with saved-pose parameters");
        }
        const bool gripperOnly = parameterOverrides.count("gripper_target") != 0;
        if (gripperOnly)
        {
            if (xyzMode || pickPlaceMode || parameterOverrides.count("pose_name") != 0 ||
                std::any_of(parameterOverrides.begin(), parameterOverrides.end(),
                    [](const auto &parameter) { return parameter.first.starts_with("poses."); }))
            {
                throw std::invalid_argument("gripper_target cannot be combined with saved-pose or XYZ/cycle targets");
            }
            firstJointIndex = 5;
            poseName = "gripper";
            targets.assign(6, 0.0);
            targets[5] = declare_parameter<double>("gripper_target", 0.0, startupDescriptor);
        }
        execute = declare_parameter<bool>("execute", !xyzMode && !pickPlaceMode && !gripperOnly, startupDescriptor);
        toleranceDegrees = declare_parameter<double>("position_tolerance_degrees", 2.0, startupDescriptor);
        joint2ToleranceDegrees = declare_parameter<double>(
            "joint2_position_tolerance_degrees", toleranceDegrees, startupDescriptor);
        if (pickPlaceMode)
        {
            if (xyzMode || !valid_xyz(pickXyz) || !valid_xyz(placeXyz))
            {
                throw std::invalid_argument("Use both pick_xyz and place_xyz with three finite coordinates each, without target_xyz");
            }
            clearanceMetres = declare_parameter<double>("clearance_metres", 0.02, startupDescriptor);
            gripperOpenPosition = declare_parameter<double>("gripper_open_position", -1.54, startupDescriptor);
            gripperClosedPosition = declare_parameter<double>("gripper_closed_position", 0.0, startupDescriptor);
            if (!std::isfinite(clearanceMetres) || clearanceMetres <= 0.0 ||
                !std::isfinite(pickXyz[2] + clearanceMetres) ||
                !std::isfinite(placeXyz[2] + clearanceMetres))
            {
                throw std::invalid_argument("clearance_metres must be finite and positive, with finite approach heights");
            }
            if (!valid_position(5, gripperOpenPosition) || !valid_position(5, gripperClosedPosition) ||
                gripperOpenPosition >= gripperClosedPosition)
            {
                throw std::invalid_argument("Gripper positions must be within [-1.54, 0.0] radians, with open below closed");
            }
            poseName = "pick_place";
        }
        if (xyzMode || pickPlaceMode)
        {
            if (xyzMode && !valid_xyz(targetXyz))
            {
                throw std::invalid_argument("target_xyz must contain three finite coordinates in metres relative to base_link");
            }
            if (xyzMode)
            {
                poseName = "xyz";
            }
            descriptionSubscription = create_subscription<std_msgs::msg::String>(
                "robot_description", rclcpp::QoS(1).transient_local().reliable(),
                [this](const std_msgs::msg::String &description) {
                    if (modelReceived && description.data != robotDescription)
                    {
                        finish(false, "Robot description changed during XYZ targeting");
                        return;
                    }
                    robotDescription = description.data;
                    modelReceived = true;
                });
        }
        else if (!gripperOnly)
        {
            targets = declare_parameter<std::vector<double>>("poses." + poseName, {}, startupDescriptor);
            if (poseName.empty() || targets.size() != 6)
            {
                throw std::invalid_argument("The selected pose must have a name and six joint positions in radians");
            }
        }
        if (!std::isfinite(toleranceDegrees) || toleranceDegrees <= 0.0 || toleranceDegrees > 2.0)
        {
            throw std::invalid_argument("position_tolerance_degrees must be greater than zero and at most 2.0");
        }
        if (!std::isfinite(joint2ToleranceDegrees) || joint2ToleranceDegrees <= 0.0 || joint2ToleranceDegrees > 2.0)
        {
            throw std::invalid_argument("joint2_position_tolerance_degrees must be greater than zero and at most 2.0");
        }
        for (std::size_t jointIndex = firstJointIndex; jointIndex < targets.size(); ++jointIndex)
        {
            if (!valid_position(jointIndex, targets[jointIndex]))
            {
                throw std::invalid_argument(std::string("Invalid target for ") + jointNames[jointIndex]);
            }
        }

        commandPublisher = create_publisher<sensor_msgs::msg::JointState>("joint_commands", 1);
        sentCommandSubscription = create_subscription<sensor_msgs::msg::JointState>(
            "joint_command_sent", 10,
            [this](const sensor_msgs::msg::JointState &command) { confirm_transmission(command); });
        feedbackSubscription = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [this](const sensor_msgs::msg::JointState &measurement) { receive_positions(measurement); });
        if (continuousMotion)
        {
            motionModeSubscription = create_subscription<std_msgs::msg::Bool>(
                "continuous_motion_enabled", rclcpp::QoS(1).transient_local().reliable(),
                [this](const std_msgs::msg::Bool &mode) {
                    motionModeReceived = true;
                    continuousDriver = mode.data;
                });
        }
        startTime = std::chrono::steady_clock::now();
        nextCommandTime = startTime + std::chrono::milliseconds(2100);
        executionTimer = create_wall_timer(std::chrono::milliseconds(50), [this]() { advance_pose(); });
        RCLCPP_INFO(get_logger(), "Pose runner initialized: target='%s'; waiting for fresh joint feedback", poseName.c_str());
        RCLCPP_INFO(get_logger(), "Joint arrival tolerances: default=%.3f degrees, arm_joint2=%.3f degrees",
            toleranceDegrees, joint2ToleranceDegrees);
        if (xyzMode)
        {
            RCLCPP_INFO(get_logger(), "%s target configured: point=%s, frame=base_link, position=[%.6f, %.6f, %.6f] m, execution=%s",
                targetFrame == "grasp_contact_frame" ? "Contact" : "Wrist", targetFrame.c_str(),
                targetXyz[0], targetXyz[1], targetXyz[2], execute ? "enabled" : "disabled");
        }
        if (gripperOnly)
        {
            RCLCPP_INFO(get_logger(), "Gripper target configured: joint=grip_joint, position=%.6f rad, execution=%s",
                targets[5], execute ? "enabled" : "disabled");
        }
        if (downwardApproach)
        {
            RCLCPP_INFO(get_logger(), "Approach configured: mode=downward, tool_axis=+Z, frame=base_link, direction=[0, 0, -1], tolerance=%.3f degrees",
                ArmKinematics::downwardToleranceDegrees);
        }
        if (approachDirection)
        {
            RCLCPP_INFO(get_logger(), "Approach configured: mode=fixed_direction, tool_axis=+Z, frame=base_link, direction=[%.6f, %.6f, %.6f], tolerance=%.3f degrees",
                (*approachDirection)[0], (*approachDirection)[1], (*approachDirection)[2],
                ArmKinematics::directionToleranceDegrees);
        }
    }

    bool finished() const
    {
        return finishedFlag;
    }

    bool succeeded() const
    {
        return successFlag;
    }

private:
    struct CycleStage
    {
        const char *name;
        std::array<double, 6> positions;
        std::array<double, 3> wristPosition;
    };

    static bool valid_xyz(const std::vector<double> &coordinates)
    {
        return coordinates.size() == 3 && std::all_of(coordinates.begin(), coordinates.end(),
            [](double value) { return std::isfinite(value); });
    }

    static double servo_degrees(std::size_t jointIndex, double positionRadians)
    {
        return jointIndex == 5
            ? 172.12 + positionRadians * (172.12 - 0.82) / 1.54
            : 90.0 + positionRadians * 180.0 / std::numbers::pi;
    }

    static bool valid_position(std::size_t jointIndex, double positionRadians)
    {
        const double lowerLimit = jointIndex == 5 ? -1.54 : -1.5708;
        const double upperLimit = jointIndex == 5 ? 0.0 : (jointIndex == 4 ? 3.14159 : 1.5708);
        if (!std::isfinite(positionRadians) || positionRadians < lowerLimit || positionRadians > upperLimit)
        {
            return false;
        }
        const double angleDegrees = servo_degrees(jointIndex, positionRadians);
        return angleDegrees >= 0.0 && angleDegrees <= (jointIndex == 4 ? 270.0 : 180.0);
    }

    static bool valid_feedback(std::size_t jointIndex, double positionRadians)
    {
        if (jointIndex != 5)
        {
            return valid_position(jointIndex, positionRadians);
        }
        const double angleDegrees = servo_degrees(jointIndex, positionRadians);
        return std::isfinite(positionRadians) && angleDegrees >= -1e-9 && angleDegrees <= 180.0 + 1e-9;
    }

    void receive_positions(const sensor_msgs::msg::JointState &measurement)
    {
        if (measurement.name.empty() || measurement.name.size() != measurement.position.size())
        {
            return;
        }
        const auto measurementStamp = rclcpp::Time(measurement.header.stamp).nanoseconds();
        const auto currentStamp = now().nanoseconds();
        if (measurementStamp <= 0 || measurementStamp > currentStamp + 100000000 ||
            currentStamp - measurementStamp > 2000000000)
        {
            return;
        }
        for (std::size_t valueIndex = 0; valueIndex < measurement.name.size(); ++valueIndex)
        {
            const auto joint = std::find(jointNames.begin(), jointNames.end(), measurement.name[valueIndex]);
            if (joint == jointNames.end())
            {
                continue;
            }
            const auto jointIndex = static_cast<std::size_t>(joint - jointNames.begin());
            if (jointIndex < firstJointIndex)
            {
                continue;
            }
            const auto previousEnd = measurement.name.begin() + valueIndex;
            if (!valid_feedback(jointIndex, measurement.position[valueIndex]) ||
                std::find(measurement.name.begin(), previousEnd, measurement.name[valueIndex]) != previousEnd)
            {
                return;
            }
        }
        const auto receivedTime = std::chrono::steady_clock::now();
        for (std::size_t valueIndex = 0; valueIndex < measurement.name.size(); ++valueIndex)
        {
            const auto joint = std::find(jointNames.begin(), jointNames.end(), measurement.name[valueIndex]);
            if (joint == jointNames.end())
            {
                continue;
            }
            const auto jointIndex = static_cast<std::size_t>(joint - jointNames.begin());
            if (jointIndex < firstJointIndex)
            {
                continue;
            }
            if (measurementStamp > lastStamp[jointIndex])
            {
                positions[jointIndex] = measurement.position[valueIndex];
                lastStamp[jointIndex] = measurementStamp;
                lastFeedbackTime[jointIndex] = receivedTime;
            }
        }
    }

    void confirm_transmission(const sensor_msgs::msg::JointState &command)
    {
        if (finishedFlag || !waitingForTransmission ||
            rclcpp::Time(command.header.stamp).nanoseconds() != commandStamp ||
            command.name.size() != jointNames.size() - firstJointIndex ||
            command.position.size() != stepTargets.size() - firstJointIndex ||
            !command.velocity.empty() || !command.effort.empty() ||
            !std::equal(command.name.begin(), command.name.end(), jointNames.begin() + firstJointIndex) ||
            !std::equal(command.position.begin(), command.position.end(), stepTargets.begin() + firstJointIndex))
        {
            return;
        }
        waitingForTransmission = false;
        feedbackAfterStamp = now().nanoseconds();
        commandTime = std::chrono::steady_clock::now();
        nextCommandTime = commandTime + std::chrono::milliseconds(continuousMotion ? 500 : 2100);
        RCLCPP_INFO(get_logger(), "Pose '%s': command transmission confirmed", poseName.c_str());
    }

    void finish(bool successful, const std::string &message)
    {
        successFlag = successful;
        finishedFlag = true;
        if (successful)
        {
            RCLCPP_INFO(get_logger(), "%s", message.c_str());
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Pose execution failed: %s", message.c_str());
        }
    }

    void log_step_errors()
    {
        for (std::size_t jointIndex = firstJointIndex; jointIndex < jointNames.size(); ++jointIndex)
        {
            const double errorDegrees = std::abs(servo_degrees(jointIndex, positions[jointIndex]) -
                servo_degrees(jointIndex, stepTargets[jointIndex]));
            const double allowedDegrees = jointIndex == 1 ? joint2ToleranceDegrees : toleranceDegrees;
            const bool feedbackAfterConfirmation = lastStamp[jointIndex] > feedbackAfterStamp;
            if (errorDegrees > allowedDegrees || !feedbackAfterConfirmation)
            {
                RCLCPP_WARN(get_logger(), "Joint step missed: joint=%s, target=%.6f rad, measured=%.6f rad, "
                    "error=%.3f servo degrees, tolerance=%.3f servo degrees, feedback_after_confirmation=%s, phase=%s",
                    jointNames[jointIndex], stepTargets[jointIndex], positions[jointIndex],
                    errorDegrees, allowedDegrees, feedbackAfterConfirmation ? "true" : "false",
                    correctionSent ? "after_correction" : "before_correction");
            }
        }
    }

    void publish_step()
    {
        sensor_msgs::msg::JointState command;
        command.header.stamp = now();
        command.name.assign(jointNames.begin() + firstJointIndex, jointNames.end());
        command.position.assign(stepTargets.begin() + firstJointIndex, stepTargets.end());
        commandStamp = rclcpp::Time(command.header.stamp).nanoseconds();
        commandTime = std::chrono::steady_clock::now();
        waitingForTransmission = true;
        waitingForStep = !continuousMotion;
        commandPublisher->publish(command);
        RCLCPP_INFO(get_logger(), "Pose '%s': joint command published", poseName.c_str());
    }

    void calculate_xyz_targets()
    {
        kinematics = std::make_unique<ArmKinematics>(robotDescription);
        std::array<double, 5> currentAngles{};
        std::copy_n(positions.begin(), currentAngles.size(), currentAngles.begin());
        std::array<double, 3> toolOffset{};
        if (targetFrame == "grasp_contact_frame")
        {
            graspKinematics = std::make_unique<GraspKinematics>(robotDescription);
            toolOffset = graspKinematics->contact_position(positions[5]);
        }
        const auto solution = kinematics->solve_position(currentAngles,
            {targetXyz[0], targetXyz[1], targetXyz[2]}, toolOffset, downwardApproach, approachDirection);
        targets.assign(positions.begin(), positions.end());
        std::copy(solution.jointPositions.begin(), solution.jointPositions.end(), targets.begin());
        for (std::size_t jointIndex = 0; jointIndex < targets.size(); ++jointIndex)
        {
            if (!valid_position(jointIndex, targets[jointIndex]))
            {
                throw std::runtime_error("IK solution exceeds the supported joint or servo limits");
            }
        }
        RCLCPP_INFO(get_logger(), "IK solution: arm=[%.6f, %.6f, %.6f, %.6f, %.6f] rad, grip_joint=%.6f rad",
            targets[0], targets[1], targets[2], targets[3], targets[4], targets[5]);
        RCLCPP_INFO(get_logger(), "IK %s position: frame=base_link, position=[%.6f, %.6f, %.6f] m, error=%.6f mm",
            graspKinematics ? "contact" : "wrist",
            solution.targetPosition[0], solution.targetPosition[1], solution.targetPosition[2],
            solution.errorMetres * 1000.0);
        if (downwardApproach)
        {
            const auto direction = kinematics->tool_direction(solution.jointPositions);
            RCLCPP_INFO(get_logger(), "IK tool direction: frame=base_link, direction=[%.6f, %.6f, %.6f], downward_error=%.6f degrees",
                direction[0], direction[1], direction[2], solution.downwardErrorDegrees);
        }
        if (approachDirection)
        {
            const auto direction = kinematics->tool_direction(solution.jointPositions);
            RCLCPP_INFO(get_logger(), "IK tool direction: frame=base_link, direction=[%.6f, %.6f, %.6f], direction_error=%.6f degrees",
                direction[0], direction[1], direction[2], solution.directionErrorDegrees);
        }
    }

    void load_cycle_stage()
    {
        const auto &stage = cycleStages[stageIndex];
        poseName = stage.name;
        targets.assign(stage.positions.begin(), stage.positions.end());
        targetXyz.assign(stage.wristPosition.begin(), stage.wristPosition.end());
        trajectoryPrepared = false;
        if (execute)
        {
            RCLCPP_INFO(get_logger(), "Cycle stage started: %zu/%zu, name=%s",
                stageIndex + 1, cycleStages.size(), stage.name);
        }
    }

    void prepare_pick_place()
    {
        kinematics = std::make_unique<ArmKinematics>(robotDescription);
        std::array<double, 5> currentAngles{};
        std::copy_n(positions.begin(), currentAngles.size(), currentAngles.begin());
        const auto initialWrist = kinematics->wrist_position(currentAngles);
        const std::array<double, 3> pick{pickXyz[0], pickXyz[1], pickXyz[2]};
        const std::array<double, 3> place{placeXyz[0], placeXyz[1], placeXyz[2]};
        const std::array<double, 3> pickAbove{pick[0], pick[1], pick[2] + clearanceMetres};
        const std::array<double, 3> placeAbove{place[0], place[1], place[2] + clearanceMetres};
        const std::array<const char *, 9> stageNames{
            "open_gripper", "pick_approach", "pick_lower", "close_gripper", "pick_lift",
            "place_approach", "place_lower", "release_gripper", "place_lift"};
        const std::array<std::array<double, 3>, 9> wristTargets{
            initialWrist, pickAbove, pick, pick, pickAbove, placeAbove, place, place, placeAbove};
        const std::array<double, 9> gripperTargets{
            gripperOpenPosition, gripperOpenPosition, gripperOpenPosition, gripperClosedPosition,
            gripperClosedPosition, gripperClosedPosition, gripperClosedPosition,
            gripperOpenPosition, gripperOpenPosition};
        cycleStages.reserve(stageNames.size());
        for (std::size_t index = 0; index < stageNames.size(); ++index)
        {
            try
            {
                const auto solution = kinematics->solve_position(currentAngles, wristTargets[index]);
                CycleStage stage{stageNames[index], {}, wristTargets[index]};
                std::copy(solution.jointPositions.begin(), solution.jointPositions.end(), stage.positions.begin());
                stage.positions[5] = gripperTargets[index];
                for (std::size_t jointIndex = 0; jointIndex < stage.positions.size(); ++jointIndex)
                {
                    if (!valid_position(jointIndex, stage.positions[jointIndex]))
                    {
                        throw std::runtime_error("IK solution exceeds the supported joint or servo limits");
                    }
                }
                cycleStages.push_back(stage);
                currentAngles = solution.jointPositions;
                RCLCPP_INFO(get_logger(), "Cycle stage planned: %zu/%zu, name=%s, wrist=[%.6f, %.6f, %.6f] m, grip_joint=%.6f rad",
                    index + 1, stageNames.size(), stage.name, stage.wristPosition[0],
                    stage.wristPosition[1], stage.wristPosition[2], stage.positions[5]);
            }
            catch (const std::exception &error)
            {
                throw std::runtime_error(std::string("Cycle planning failed at '") + stageNames[index] + "': " + error.what());
            }
        }
        load_cycle_stage();
    }

    void complete_pose()
    {
        if (kinematics)
        {
            try
            {
                std::array<double, 5> currentAngles{};
                std::copy_n(positions.begin(), currentAngles.size(), currentAngles.begin());
                const auto measuredPosition = graspKinematics ?
                    kinematics->target_position(currentAngles, graspKinematics->contact_position(positions[5])) :
                    kinematics->wrist_position(currentAngles);
                const double errorMetres = std::hypot(measuredPosition[0] - targetXyz[0],
                    measuredPosition[1] - targetXyz[1], measuredPosition[2] - targetXyz[2]);
                RCLCPP_INFO(get_logger(), "%s position from joint feedback: frame=base_link, position=[%.6f, %.6f, %.6f] m, error=%.6f mm",
                    graspKinematics ? "Contact" : "Wrist", measuredPosition[0], measuredPosition[1],
                    measuredPosition[2], errorMetres * 1000.0);
                if (errorMetres > 0.001)
                {
                    finish(false, graspKinematics ?
                        "Feedback contact position missed the XYZ target by more than 1 mm" :
                        "Feedback wrist position missed the XYZ target by more than 1 mm");
                    return;
                }
                if (downwardApproach || approachDirection)
                {
                    const auto direction = kinematics->tool_direction(currentAngles);
                    const double directionError = kinematics->direction_error_degrees(currentAngles,
                        approachDirection.value_or(std::array<double, 3>{0.0, 0.0, -1.0}));
                    RCLCPP_INFO(get_logger(), "Tool direction from joint feedback: frame=base_link, direction=[%.6f, %.6f, %.6f], direction_error=%.6f degrees",
                        direction[0], direction[1], direction[2], directionError);
                    if (!std::isfinite(directionError) || directionError > ArmKinematics::directionToleranceDegrees)
                    {
                        finish(false, downwardApproach ? "Feedback tool direction missed downward by more than 1 degree" :
                            "Feedback tool direction missed requested direction by more than 1 degree");
                        return;
                    }
                }
            }
            catch (const std::exception &error)
            {
                finish(false, error.what());
                return;
            }
        }
        if (pickPlaceMode)
        {
            RCLCPP_INFO(get_logger(), "Cycle stage reached: %zu/%zu, name=%s",
                stageIndex + 1, cycleStages.size(), poseName.c_str());
            if (++stageIndex < cycleStages.size())
            {
                load_cycle_stage();
                return;
            }
            finish(true, "Pick-and-place motion cycle completed");
        }
        else
        {
            finish(true, firstJointIndex == 5 ? "Gripper target reached within joint tolerance" :
                "Pose '" + poseName + "' reached within joint tolerance");
        }
    }

    void prepare_continuous_motion()
    {
        trajectoryStartPositions = positions;
        trajectoryDurationSeconds = 1.0;
        for (std::size_t jointIndex = firstJointIndex; jointIndex < targets.size(); ++jointIndex)
        {
            const double distanceDegrees = std::abs(servo_degrees(jointIndex, targets[jointIndex]) -
                servo_degrees(jointIndex, positions[jointIndex]));
            const double velocityLimit = jointIndex == 5 ? 3.0 : 1.5;
            const double accelerationLimit = jointIndex == 5 ? 2.0 : 1.0;
            trajectoryDurationSeconds = std::max({trajectoryDurationSeconds,
                1.875 * distanceDegrees / velocityLimit,
                std::sqrt(5.7735026919 * distanceDegrees / accelerationLimit)});
        }
        trajectoryStartTime = std::chrono::steady_clock::now();
        nextCommandTime = trajectoryStartTime + std::chrono::milliseconds(500);
        finalCommandSent = false;
        trajectoryPrepared = true;
        RCLCPP_INFO(get_logger(), "Continuous trajectory planned: target='%s', duration=%.3f s, "
            "interval=500 ms, arm_velocity_limit=1.5 degrees/s, gripper_velocity_limit=3 degrees/s",
            poseName.c_str(), trajectoryDurationSeconds);
    }

    void advance_continuous_motion()
    {
        if (!trajectoryPrepared)
        {
            prepare_continuous_motion();
            return;
        }
        const auto currentTime = std::chrono::steady_clock::now();
        if (finalCommandSent)
        {
            bool arrived = true;
            for (std::size_t jointIndex = firstJointIndex; jointIndex < targets.size(); ++jointIndex)
            {
                const double errorDegrees = std::abs(servo_degrees(jointIndex, targets[jointIndex]) -
                    servo_degrees(jointIndex, positions[jointIndex]));
                if (lastStamp[jointIndex] <= feedbackAfterStamp ||
                    errorDegrees > (jointIndex == 1 ? joint2ToleranceDegrees : toleranceDegrees))
                {
                    arrived = false;
                }
            }
            if (currentTime - commandTime >= std::chrono::seconds(1) && arrived)
            {
                complete_pose();
            }
            else if (currentTime - commandTime >= std::chrono::seconds(5))
            {
                log_step_errors();
                finish(false, "Continuous trajectory final target timed out (5000 ms); no retry sent");
            }
            return;
        }
        if (currentTime < nextCommandTime)
        {
            return;
        }
        for (std::size_t jointIndex = firstJointIndex; jointIndex < targets.size(); ++jointIndex)
        {
            if (commandStamp != 0 && lastStamp[jointIndex] <= feedbackAfterStamp)
            {
                return;
            }
        }
        const double elapsedSeconds = std::chrono::duration<double>(currentTime - trajectoryStartTime).count();
        const double progress = std::clamp(elapsedSeconds / trajectoryDurationSeconds, 0.0, 1.0);
        const double fraction = progress * progress * progress *
            (10.0 + progress * (-15.0 + 6.0 * progress));
        for (std::size_t jointIndex = firstJointIndex; jointIndex < targets.size(); ++jointIndex)
        {
            stepTargets[jointIndex] = progress >= 1.0 ? targets[jointIndex] :
                trajectoryStartPositions[jointIndex] +
                (targets[jointIndex] - trajectoryStartPositions[jointIndex]) * fraction;
            const double errorDegrees = std::abs(servo_degrees(jointIndex, stepTargets[jointIndex]) -
                servo_degrees(jointIndex, positions[jointIndex]));
            if (!valid_position(jointIndex, stepTargets[jointIndex]) ||
                errorDegrees > (jointIndex == 5 ? 4.5 : 2.5))
            {
                RCLCPP_WARN(get_logger(), "Continuous tracking limit exceeded: joint=%s, "
                    "target=%.6f rad, measured=%.6f rad, error=%.3f servo degrees",
                    jointNames[jointIndex], stepTargets[jointIndex], positions[jointIndex], errorDegrees);
                finish(false, "Continuous trajectory stopped before sending an out-of-range command; no retry sent");
                return;
            }
        }
        finalCommandSent = progress >= 1.0;
        publish_step();
    }

    void advance_pose()
    {
        if (finishedFlag)
        {
            return;
        }
        const auto currentTime = std::chrono::steady_clock::now();
        const bool freshFeedback = std::all_of(lastFeedbackTime.begin() + firstJointIndex, lastFeedbackTime.end(),
            [currentTime](const auto &receivedTime) {
                return receivedTime.time_since_epoch().count() != 0 &&
                    currentTime - receivedTime <= std::chrono::seconds(2);
            });
        if (!ready)
        {
            if (execute && continuousMotion && motionModeReceived && !continuousDriver)
            {
                finish(false, "Continuous motion requires continuous_motion:=true on the driver");
                return;
            }
            if (freshFeedback && (!execute || (commandPublisher->get_subscription_count() > 0 &&
                sentCommandSubscription->get_publisher_count() > 0)) &&
                (!execute || !continuousMotion || (motionModeReceived && continuousDriver &&
                    motionModeSubscription->get_publisher_count() == 1)) &&
                ((!pickPlaceMode && targetXyz.empty()) || modelReceived))
            {
                if (pickPlaceMode || !targetXyz.empty())
                {
                    try
                    {
                        if (pickPlaceMode)
                        {
                            prepare_pick_place();
                        }
                        else
                        {
                            calculate_xyz_targets();
                        }
                    }
                    catch (const std::exception &error)
                    {
                        finish(false, error.what());
                        return;
                    }
                }
                ready = true;
                if (continuousMotion)
                {
                    prepare_continuous_motion();
                }
                if (!execute)
                {
                    finish(true, pickPlaceMode ? "Pick-and-place plan validated: execution disabled" :
                        "Target validation completed: execution disabled");
                }
            }
            else if (currentTime - startTime >= std::chrono::seconds(5))
            {
                if (execute && continuousMotion && (!motionModeReceived ||
                    motionModeSubscription->get_publisher_count() != 1))
                {
                    finish(false, "Continuous motion requires exactly one driver mode publisher");
                    return;
                }
                finish(false, firstJointIndex == 5 ?
                    "No fresh gripper feedback or required command endpoints available" :
                    !pickPlaceMode && targetXyz.empty() ?
                    "No command receiver, transmission confirmation publisher or complete fresh feedback available" :
                    "No robot description, complete fresh feedback or command endpoints available for XYZ targeting");
            }
            return;
        }
        if (!freshFeedback || commandPublisher->get_subscription_count() == 0 ||
            sentCommandSubscription->get_publisher_count() == 0)
        {
            finish(false, firstJointIndex == 5 ?
                "Gripper feedback became stale or a command endpoint disconnected" :
                "Joint feedback became stale or a command endpoint disconnected");
            return;
        }
        if (continuousMotion && (!continuousDriver || motionModeSubscription->get_publisher_count() != 1))
        {
            finish(false, "Continuous driver mode changed or its publisher disconnected");
            return;
        }
        if (waitingForTransmission)
        {
            if (currentTime - commandTime >= std::chrono::seconds(5))
            {
                finish(false, "Command transmission confirmation timed out (5000 ms)");
            }
            return;
        }
        if (continuousMotion)
        {
            advance_continuous_motion();
            return;
        }
        if (waitingForStep)
        {
            bool stepReached = true;
            for (std::size_t jointIndex = firstJointIndex; jointIndex < jointNames.size(); ++jointIndex)
            {
                if (lastStamp[jointIndex] <= feedbackAfterStamp ||
                    std::abs(servo_degrees(jointIndex, positions[jointIndex]) -
                        servo_degrees(jointIndex, stepTargets[jointIndex])) >
                        (jointIndex == 1 ? joint2ToleranceDegrees : toleranceDegrees))
                {
                    stepReached = false;
                }
            }
            if (currentTime - commandTime >= std::chrono::seconds(1) &&
                stepReached)
            {
                waitingForStep = false;
            }
            else
            {
                if (currentTime - commandTime >= std::chrono::seconds(5))
                {
                    log_step_errors();
                    if (correctionSent)
                    {
                        finish(false, "Joint target step timed out after one correction (5000 ms)");
                        return;
                    }
                    double maximumErrorDegrees = 0.0;
                    for (std::size_t jointIndex = firstJointIndex; jointIndex < jointNames.size(); ++jointIndex)
                    {
                        const double errorDegrees = std::abs(servo_degrees(jointIndex, positions[jointIndex]) -
                            servo_degrees(jointIndex, stepTargets[jointIndex]));
                        if (lastStamp[jointIndex] <= feedbackAfterStamp ||
                            errorDegrees > (jointIndex == 5 ? 4.5 : 2.5))
                        {
                            finish(false, "Joint correction cancelled: feedback predates confirmation or correction exceeds step limits");
                            return;
                        }
                        maximumErrorDegrees = std::max(maximumErrorDegrees, errorDegrees);
                    }
                    correctionSent = true;
                    RCLCPP_WARN(get_logger(), "Joint step outside tolerance: largest_error=%.3f degrees; correction_attempt=1",
                        maximumErrorDegrees);
                    publish_step();
                }
                return;
            }
        }
        if (currentTime < nextCommandTime)
        {
            return;
        }
        bool poseReached = true;
        double stepFraction = 1.0;
        for (std::size_t jointIndex = firstJointIndex; jointIndex < jointNames.size(); ++jointIndex)
        {
            const double remainingDegrees = std::abs(servo_degrees(jointIndex, targets[jointIndex]) -
                servo_degrees(jointIndex, positions[jointIndex]));
            if (remainingDegrees > (jointIndex == 1 ? joint2ToleranceDegrees : toleranceDegrees))
            {
                poseReached = false;
                stepFraction = std::min(stepFraction, (jointIndex == 5 ? 4.5 : 2.5) / remainingDegrees);
            }
        }
        if (poseReached)
        {
            complete_pose();
            return;
        }

        for (std::size_t jointIndex = firstJointIndex; jointIndex < jointNames.size(); ++jointIndex)
        {
            stepTargets[jointIndex] = positions[jointIndex] +
                (targets[jointIndex] - positions[jointIndex]) * stepFraction;
            if (!valid_position(jointIndex, stepTargets[jointIndex]))
            {
                finish(false, std::string("Cannot reach the command range for ") + jointNames[jointIndex] +
                    " within a bounded step; check calibration and measured position");
                return;
            }
        }
        correctionSent = false;
        publish_step();
    }

    static constexpr std::array<const char *, 6> jointNames{
        "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "grip_joint"};
    std::string poseName;
    std::string targetFrame;
    std::size_t firstJointIndex = 0;
    std::vector<double> targets;
    std::vector<double> targetXyz;
    std::vector<double> pickXyz;
    std::vector<double> placeXyz;
    bool pickPlaceMode = false;
    bool downwardApproach = false;
    std::optional<std::array<double, 3>> approachDirection;
    double clearanceMetres = 0.02;
    double gripperOpenPosition = -1.54;
    double gripperClosedPosition = 0.0;
    std::vector<CycleStage> cycleStages;
    std::size_t stageIndex = 0;
    std::string robotDescription;
    std::unique_ptr<ArmKinematics> kinematics;
    std::unique_ptr<GraspKinematics> graspKinematics;
    bool modelReceived = false;
    bool execute = true;
    bool continuousMotion = false;
    bool motionModeReceived = false;
    bool continuousDriver = false;
    bool trajectoryPrepared = false;
    bool finalCommandSent = false;
    double trajectoryDurationSeconds = 1.0;
    std::array<double, 6> trajectoryStartPositions{};
    std::chrono::steady_clock::time_point trajectoryStartTime{};
    double toleranceDegrees = 2.0;
    double joint2ToleranceDegrees = 2.0;
    std::array<double, 6> positions{};
    std::array<std::int64_t, 6> lastStamp{};
    std::array<std::chrono::steady_clock::time_point, 6> lastFeedbackTime{};
    std::array<double, 6> stepTargets{};
    std::int64_t commandStamp = 0;
    std::int64_t feedbackAfterStamp = 0;
    bool ready = false;
    bool waitingForTransmission = false;
    bool waitingForStep = false;
    bool correctionSent = false;
    bool finishedFlag = false;
    bool successFlag = false;
    std::chrono::steady_clock::time_point startTime{};
    std::chrono::steady_clock::time_point commandTime{};
    std::chrono::steady_clock::time_point nextCommandTime{};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr commandPublisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sentCommandSubscription;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedbackSubscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr descriptionSubscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motionModeSubscription;
    rclcpp::TimerBase::SharedPtr executionTimer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    int exitCode = 1;
    try
    {
        auto poseRunner = std::make_shared<PoseRunner>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(poseRunner);
        while (rclcpp::ok() && !poseRunner->finished())
        {
            executor.spin_once(std::chrono::milliseconds(100));
        }
        exitCode = poseRunner->succeeded() ? 0 : 1;
        if (!poseRunner->finished())
        {
            RCLCPP_WARN(poseRunner->get_logger(), "Pose execution interrupted");
        }
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("pose_runner"), "%s", error.what());
    }
    rclcpp::shutdown();
    return exitCode;
}
