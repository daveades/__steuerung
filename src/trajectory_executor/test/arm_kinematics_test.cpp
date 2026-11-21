#include "arm_kinematics.hpp"
#include <dofbot_description/grasp_kinematics.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>

static std::string robot_description()
{
    const auto path = ament_index_cpp::get_package_share_directory("dofbot_description") + "/urdf/dofbot.urdf";
    std::ifstream input(path);
    if (!input)
    {
        throw std::runtime_error("Could not read the test robot model");
    }
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

TEST(ArmKinematics, MatchesZeroAngleGeometry)
{
    ArmKinematics kinematics(robot_description());
    const auto position = kinematics.wrist_position({});
    EXPECT_NEAR(position[0], -0.00215, 1e-12);
    EXPECT_NEAR(position[1], 0.0041162542814959825, 1e-12);
    EXPECT_NEAR(position[2], 0.4468777549634534, 1e-12);
}

TEST(ArmKinematics, KeepsAnAlreadyReachedConfiguration)
{
    ArmKinematics kinematics(robot_description());
    const std::array<double, 5> currentAngles{0.2, 0.3, -0.4, 0.15, 0.7};
    const auto solution = kinematics.solve_position(currentAngles, kinematics.wrist_position(currentAngles));
    for (std::size_t jointIndex = 0; jointIndex < currentAngles.size(); ++jointIndex)
    {
        EXPECT_NEAR(solution.jointPositions[jointIndex], currentAngles[jointIndex], 1e-9);
    }
    EXPECT_LT(solution.errorMetres, 1e-9);
}

TEST(ArmKinematics, SolvesFromAStraightArmAndHoldsWristRotation)
{
    ArmKinematics kinematics(robot_description());
    const std::array<double, 5> start{0.0, 0.0, 0.0, 0.0, 0.6};
    const auto solution = kinematics.solve_position(start, {0.02, 0.0, 0.42});
    EXPECT_LE(solution.errorMetres, 0.001);
    EXPECT_DOUBLE_EQ(solution.jointPositions[4], start[4]);
    for (std::size_t jointIndex = 0; jointIndex < 4; ++jointIndex)
    {
        EXPECT_GE(solution.jointPositions[jointIndex], -1.5708);
        EXPECT_LE(solution.jointPositions[jointIndex], 1.5708);
    }
}

TEST(ArmKinematics, RejectsTargetsBeyondTheArmReach)
{
    ArmKinematics kinematics(robot_description());
    EXPECT_THROW(kinematics.solve_position({}, {2.0, 2.0, 2.0}), std::runtime_error);
}

TEST(ArmKinematics, RejectsInvalidNumbersAndCurrentAngles)
{
    ArmKinematics kinematics(robot_description());
    const auto nanValue = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(kinematics.solve_position({}, {nanValue, 0.0, 0.4}), std::invalid_argument);
    EXPECT_THROW(kinematics.wrist_position({nanValue, 0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
    EXPECT_THROW(kinematics.solve_position({2.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.4}),
        std::invalid_argument);
}

TEST(ArmKinematics, EnforcesLimitsReadFromTheUrdf)
{
    auto description = robot_description();
    std::size_t index = 0;
    while ((index = description.find("lower=\"-1.5708\"", index)) != std::string::npos)
    {
        description.replace(index, 15, "lower=\"-0.01\"");
        index += 13;
    }
    index = 0;
    while ((index = description.find("upper=\"1.5708\"", index)) != std::string::npos)
    {
        description.replace(index, 14, "upper=\"0.01\"");
        index += 12;
    }
    ArmKinematics kinematics(description);
    EXPECT_THROW(kinematics.solve_position({}, {0.02, 0.0, 0.42}), std::runtime_error);
    EXPECT_THROW(kinematics.wrist_position({0.02, 0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
}

TEST(ArmKinematics, RejectsAnInvalidModelOrMissingWristChain)
{
    EXPECT_THROW(ArmKinematics("invalid"), std::invalid_argument);
    auto description = robot_description();
    std::size_t index = 0;
    while ((index = description.find("arm_link5", index)) != std::string::npos)
    {
        description.replace(index, 9, "wrist_link");
        index += 10;
    }
    EXPECT_THROW(ArmKinematics{description}, std::invalid_argument);
}

TEST(GraspKinematics, MatchesIndependentLinkageGeometry)
{
    GraspKinematics kinematics(robot_description());
    const std::array<double, 3> openings{0.0, -0.8949375364856976, -1.54};
    const std::array<std::array<double, 3>, 3> expected{{
        {-0.0005, -0.00018, 0.001},
        {-0.0004999567253401161, -0.00016066992475697304, -0.010781171665115736},
        {-0.0004998906171384217, -0.00012996460907513233, -0.028778588045087518}}};
    for (std::size_t index = 0; index < openings.size(); ++index)
    {
        const auto point = kinematics.contact_position(openings[index]);
        for (std::size_t axis = 0; axis < point.size(); ++axis)
        {
            EXPECT_NEAR(point[axis], expected[index][axis], 1e-12);
        }
    }
    EXPECT_THROW(kinematics.contact_position(std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
}

TEST(ArmKinematics, RotatesTheContactOffsetWithTheArm)
{
    const auto description = robot_description();
    ArmKinematics kinematics(description);
    GraspKinematics grasp(description);
    const auto point = kinematics.target_position({0.2, 0.3, -0.4, 0.15, 0.7},
        grasp.contact_position(-0.8949375364856976));
    EXPECT_NEAR(point[0], -0.02621027482140294, 1e-12);
    EXPECT_NEAR(point[1], -0.0018149963231949041, 1e-12);
    EXPECT_NEAR(point[2], 0.43171696260371506, 1e-12);
}

TEST(ArmKinematics, SolvesForContactWithWristRotationHeld)
{
    const auto description = robot_description();
    ArmKinematics kinematics(description);
    GraspKinematics grasp(description);
    const std::array<double, 5> start{0.2, 0.3, -0.4, 0.15, 0.7};
    const auto offset = grasp.contact_position(-0.8949375364856976);
    auto target = kinematics.target_position(start, offset);
    target[0] += 0.01;
    target[2] -= 0.005;
    const auto solution = kinematics.solve_position(start, target, offset);
    EXPECT_DOUBLE_EQ(solution.jointPositions[4], start[4]);
    EXPECT_LT(solution.errorMetres, 1e-5);
    const auto point = kinematics.target_position(solution.jointPositions, offset);
    for (std::size_t axis = 0; axis < target.size(); ++axis)
    {
        EXPECT_NEAR(point[axis], target[axis], 1e-5);
    }
    EXPECT_GT(std::hypot(solution.wristPosition[0] - target[0],
        solution.wristPosition[1] - target[1], solution.wristPosition[2] - target[2]), 0.01);
}

TEST(ArmKinematics, RejectsInvalidOffsetsAndUnreachableContactTargets)
{
    ArmKinematics kinematics(robot_description());
    const auto nanValue = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(kinematics.target_position({}, {nanValue, 0.0, 0.0}), std::invalid_argument);
    EXPECT_THROW(kinematics.solve_position({}, {0.0, 0.0, 0.4}, {0.0, nanValue, 0.0}),
        std::invalid_argument);
    EXPECT_THROW(kinematics.solve_position({}, {2.0, 2.0, 2.0}, {0.0, 0.0, -0.02}),
        std::runtime_error);
}

TEST(ArmKinematics, DescribesTheFingerDirectionFromTheModel)
{
    ArmKinematics kinematics(robot_description());
    const auto direction = kinematics.tool_direction({});
    EXPECT_NEAR(direction[0], 0.0, 1e-12);
    EXPECT_NEAR(direction[1], std::sin(0.010805), 1e-12);
    EXPECT_NEAR(direction[2], std::cos(0.010805), 1e-12);
    EXPECT_GT(kinematics.downward_error_degrees({}), 179.0);
    EXPECT_LT(kinematics.downward_error_degrees({0.0, -0.15, -1.45, -1.541592653589793, 0.0}),
        ArmKinematics::downwardToleranceDegrees);
}

TEST(ArmKinematics, SolvesDownwardContactFromTheStraightPose)
{
    const auto description = robot_description();
    ArmKinematics kinematics(description);
    GraspKinematics grasp(description);
    const std::array<double, 3> target{0.10, 0.0, 0.02};
    const auto solution = kinematics.solve_position({}, target, grasp.contact_position(-0.895), true);
    EXPECT_LE(solution.errorMetres, 0.001);
    EXPECT_LE(solution.downwardErrorDegrees, ArmKinematics::downwardToleranceDegrees);
    EXPECT_DOUBLE_EQ(solution.jointPositions[4], 0.0);
    const auto direction = kinematics.tool_direction(solution.jointPositions);
    EXPECT_LT(direction[2], -0.9998);
    for (std::size_t jointIndex = 0; jointIndex < 4; ++jointIndex)
    {
        EXPECT_GE(solution.jointPositions[jointIndex], -1.5708);
        EXPECT_LE(solution.jointPositions[jointIndex], 1.5708);
    }
}

TEST(ArmKinematics, KeepsAnAlreadyReachedDownwardPoseWithTwistedWrist)
{
    const auto description = robot_description();
    ArmKinematics kinematics(description);
    GraspKinematics grasp(description);
    const std::array<double, 5> start{0.25, -0.15, -1.45, -1.541592653589793, 0.6};
    const auto offset = grasp.contact_position(-0.895);
    const auto solution = kinematics.solve_position(start, kinematics.target_position(start, offset), offset, true);
    EXPECT_LT(solution.errorMetres, 1e-9);
    EXPECT_LE(solution.downwardErrorDegrees, ArmKinematics::downwardToleranceDegrees);
    for (std::size_t jointIndex = 0; jointIndex < start.size(); ++jointIndex)
    {
        EXPECT_DOUBLE_EQ(solution.jointPositions[jointIndex], start[jointIndex]);
    }
}

TEST(ArmKinematics, RejectsPositionReachableButDirectionUnreachableTarget)
{
    const auto description = robot_description();
    ArmKinematics kinematics(description);
    GraspKinematics grasp(description);
    const auto offset = grasp.contact_position(-0.895);
    const auto target = kinematics.target_position({}, offset);
    EXPECT_LT(kinematics.solve_position({}, target, offset).errorMetres, 1e-9);
    EXPECT_THROW(kinematics.solve_position({}, target, offset, true), std::runtime_error);
}

TEST(ArmKinematics, PreservesTaughtDirectionAcrossLiftAndDescent)
{
    const auto description = robot_description();
    ArmKinematics kinematics(description);
    GraspKinematics grasp(description);
    const std::array<double, 5> taught{0.284171335483804, -0.9767497159342813,
        -0.8253820698976818, -0.6497384806287982, 0.0004731314237333697};
    const auto offset = grasp.contact_position(-1.0648499708114418);
    const auto contact = kinematics.target_position(taught, offset);
    const auto direction = kinematics.tool_direction(taught);
    auto current = taught;
    for (double height : {0.005, 0.015, 0.03, 0.02, 0.01, 0.0})
    {
        auto target = contact;
        target[2] += height;
        const auto solution = kinematics.solve_position(current, target, offset, false, direction);
        EXPECT_LE(solution.errorMetres, 0.001);
        EXPECT_LE(solution.directionErrorDegrees, ArmKinematics::directionToleranceDegrees);
        EXPECT_LE(kinematics.direction_error_degrees(solution.jointPositions, direction),
            ArmKinematics::directionToleranceDegrees);
        EXPECT_DOUBLE_EQ(solution.jointPositions[4], taught[4]);
        current = solution.jointPositions;
    }
}

TEST(ArmKinematics, KeepsReachedTaughtPoseAndNormalizesDirection)
{
    const auto description = robot_description();
    ArmKinematics kinematics(description);
    GraspKinematics grasp(description);
    const std::array<double, 5> taught{0.284171335483804, -0.9767497159342813,
        -0.8253820698976818, -0.6497384806287982, 0.7};
    const auto offset = grasp.contact_position(-1.0648499708114418);
    auto direction = kinematics.tool_direction(taught);
    for (auto &value : direction)
    {
        value *= 5.0;
    }
    const auto solution = kinematics.solve_position(taught, kinematics.target_position(taught, offset),
        offset, false, direction);
    EXPECT_LT(solution.errorMetres, 1e-9);
    EXPECT_LT(solution.directionErrorDegrees, 1e-5);
    for (std::size_t index = 0; index < taught.size(); ++index)
    {
        EXPECT_DOUBLE_EQ(solution.jointPositions[index], taught[index]);
    }
}

TEST(ArmKinematics, RejectsInvalidAndConflictingDirections)
{
    ArmKinematics kinematics(robot_description());
    const auto nanValue = std::numeric_limits<double>::quiet_NaN();
    const auto infinity = std::numeric_limits<double>::infinity();
    for (const std::array<double, 3> direction : {std::array<double, 3>{0.0, 0.0, 0.0},
        {nanValue, 0.0, -1.0}, {0.0, infinity, -1.0}})
    {
        EXPECT_THROW(kinematics.solve_position({}, {0.0, 0.0, 0.4}, {}, false, direction),
            std::invalid_argument);
        EXPECT_THROW(kinematics.direction_error_degrees({}, direction), std::invalid_argument);
    }
    EXPECT_THROW(kinematics.solve_position({}, {0.0, 0.0, 0.4}, {}, true,
        std::array<double, 3>{0.0, 0.0, -1.0}), std::invalid_argument);
}

TEST(ArmKinematics, RejectsPositionReachableTargetWithIncompatibleTaughtDirection)
{
    const auto description = robot_description();
    ArmKinematics kinematics(description);
    GraspKinematics grasp(description);
    const std::array<double, 5> taught{0.284171335483804, -0.9767497159342813,
        -0.8253820698976818, -0.6497384806287982, 0.0004731314237333697};
    auto otherPose = taught;
    otherPose[0] += 0.3;
    const auto offset = grasp.contact_position(-1.0648499708114418);
    const auto target = kinematics.target_position(otherPose, offset);
    EXPECT_LT(kinematics.solve_position(otherPose, target, offset).errorMetres, 1e-9);
    EXPECT_THROW(kinematics.solve_position(taught, target, offset, false,
        kinematics.tool_direction(taught)), std::runtime_error);
}
