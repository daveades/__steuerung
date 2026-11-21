#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>

class SimulatedArm : public rclcpp::Node
{
public:
    SimulatedArm()
        : Node("simulated_arm")
    {
        continuousMotion = declare_parameter<bool>("continuous_motion", false);
        feedbackPublisher = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        sentCommandPublisher = create_publisher<sensor_msgs::msg::JointState>("joint_command_sent", 10);
        motionModePublisher = create_publisher<std_msgs::msg::Bool>(
            "continuous_motion_enabled", rclcpp::QoS(1).transient_local().reliable());
        std_msgs::msg::Bool motionMode;
        motionMode.data = continuousMotion;
        motionModePublisher->publish(motionMode);
        commandSubscription = create_subscription<sensor_msgs::msg::JointState>(
            "joint_commands", rclcpp::QoS(1),
            [this](const sensor_msgs::msg::JointState &command) { command_joint(command); });
        feedbackTimer = create_wall_timer(
            std::chrono::milliseconds(20), [this]() { publish_positions(); });
        RCLCPP_INFO(get_logger(), "Simulated joint feedback enabled: interval=20 ms");
    }

private:
    static double servo_degrees(std::size_t jointIndex, double positionRadians)
    {
        return jointIndex == 5
            ? 172.12 + positionRadians * (172.12 - 0.82) / 1.54
            : 90.0 + positionRadians * 180.0 / std::numbers::pi;
    }

    void command_joint(const sensor_msgs::msg::JointState &command)
    {
        try
        {
            if ((command.name.size() != 1 && command.name.size() != 6) ||
                command.position.size() != command.name.size() ||
                !command.velocity.empty() || !command.effort.empty())
            {
                throw std::invalid_argument("Invalid joint command: expected one joint or all six with matching positions and no velocity or effort values");
            }
            auto requestedPositions = positions;
            std::array<bool, 6> requested{};
            for (std::size_t valueIndex = 0; valueIndex < command.name.size(); ++valueIndex)
            {
                const auto joint = std::find(jointNames.begin(), jointNames.end(), command.name[valueIndex]);
                if (joint == jointNames.end())
                {
                    throw std::invalid_argument("Unsupported joint name: expected arm_joint1 through arm_joint5 or grip_joint");
                }
                const auto jointIndex = static_cast<std::size_t>(joint - jointNames.begin());
                if (requested[jointIndex])
                {
                    throw std::invalid_argument("Invalid joint command: duplicate joint name");
                }
                requested[jointIndex] = true;
                const double requestedPosition = command.position[valueIndex];
                const double lowerLimit = jointIndex == 5 ? -1.54 : -1.5708;
                const double upperLimit = jointIndex == 5 ? 0.0 : (jointIndex == 4 ? 3.14159 : 1.5708);
                if (!std::isfinite(requestedPosition) || requestedPosition < lowerLimit || requestedPosition > upperLimit)
                {
                    throw std::invalid_argument("Target is outside the model joint limits");
                }
                const double requestedDegrees = servo_degrees(jointIndex, requestedPosition);
                if (requestedDegrees < 0.0 || requestedDegrees > (jointIndex == 4 ? 270.0 : 180.0))
                {
                    throw std::invalid_argument("Target is outside the servo angle range");
                }
                const double maximumStepDegrees = jointIndex == 5 ? 5.0 : 3.0;
                if (std::abs(requestedDegrees - servo_degrees(jointIndex, positions[jointIndex])) > maximumStepDegrees)
                {
                    throw std::invalid_argument("Command step exceeds servo limits: arm=3 degrees, gripper=5 degrees");
                }
                requestedPositions[jointIndex] = requestedPosition;
            }
            const auto commandTime = std::chrono::steady_clock::now();
            if (commandTime < nextCommandTime)
            {
                throw std::runtime_error("Movement command cooldown active: minimum interval=" +
                    std::to_string(continuousMotion ? 500 : 2000) + " ms");
            }
            startPositions = positions;
            targetPositions = requestedPositions;
            movementStart = commandTime;
            nextCommandTime = commandTime + std::chrono::milliseconds(continuousMotion ? 500 : 2000);
            moving = true;
            sentCommandPublisher->publish(command);
            RCLCPP_INFO(get_logger(), "Simulated command accepted: targets=%zu, duration=1000 ms", command.name.size());
        }
        catch (const std::exception &error)
        {
            RCLCPP_WARN(get_logger(), "Joint command failed: %s", error.what());
        }
    }

    void publish_positions()
    {
        if (moving)
        {
            const double elapsedSeconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - movementStart).count();
            const double progress = std::clamp(elapsedSeconds, 0.0, 1.0);
            for (std::size_t jointIndex = 0; jointIndex < positions.size(); ++jointIndex)
            {
                positions[jointIndex] = startPositions[jointIndex] +
                    (targetPositions[jointIndex] - startPositions[jointIndex]) * progress;
            }
            if (progress >= 1.0)
            {
                positions = targetPositions;
                moving = false;
            }
        }
        sensor_msgs::msg::JointState measurement;
        measurement.header.stamp = now();
        measurement.name.assign(jointNames.begin(), jointNames.end());
        measurement.position.assign(positions.begin(), positions.end());
        feedbackPublisher->publish(measurement);
    }

    static constexpr std::array<const char *, 6> jointNames{
        "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "grip_joint"};
    std::array<double, 6> positions{};
    std::array<double, 6> startPositions{};
    std::array<double, 6> targetPositions{};
    bool moving = false;
    bool continuousMotion = false;
    std::chrono::steady_clock::time_point movementStart{};
    std::chrono::steady_clock::time_point nextCommandTime{};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedbackPublisher;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sentCommandPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motionModePublisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr commandSubscription;
    rclcpp::TimerBase::SharedPtr feedbackTimer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    int exitCode = 0;
    try
    {
        rclcpp::spin(std::make_shared<SimulatedArm>());
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("simulated_arm"), "%s", error.what());
        exitCode = 1;
    }
    rclcpp::shutdown();
    return exitCode;
}
