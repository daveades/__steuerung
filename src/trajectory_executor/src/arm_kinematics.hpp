#pragma once

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <array>
#include <optional>
#include <string>

struct ArmSolution
{
    std::array<double, 5> jointPositions;
    std::array<double, 3> wristPosition;
    std::array<double, 3> targetPosition;
    double errorMetres;
    double downwardErrorDegrees;
    double directionErrorDegrees;
};

class ArmKinematics
{
public:
    static constexpr double downwardToleranceDegrees = 1.0;
    static constexpr double directionToleranceDegrees = 1.0;

    explicit ArmKinematics(const std::string &robotDescription);

    ArmSolution solve_position(const std::array<double, 5> &currentPositions,
        const std::array<double, 3> &targetPosition,
        const std::array<double, 3> &toolOffset = {}, bool downwardApproach = false,
        const std::optional<std::array<double, 3>> &targetDirection = std::nullopt) const;

    std::array<double, 3> wrist_position(const std::array<double, 5> &jointPositions) const;
    std::array<double, 3> target_position(const std::array<double, 5> &jointPositions,
        const std::array<double, 3> &toolOffset) const;
    std::array<double, 3> tool_direction(const std::array<double, 5> &jointPositions) const;
    double downward_error_degrees(const std::array<double, 5> &jointPositions) const;
    double direction_error_degrees(const std::array<double, 5> &jointPositions,
        const std::array<double, 3> &targetDirection) const;

private:
    KDL::Frame wrist_frame(const std::array<double, 5> &jointPositions) const;
    static KDL::Vector normalized_direction(const std::array<double, 3> &direction);
    bool solve_direction(const KDL::Chain &solveChain, const KDL::JntArray &seed,
        const KDL::Frame &targetFrame, const KDL::Vector &targetDirection, KDL::JntArray &solved) const;
    KDL::Chain chain;
    std::array<double, 5> lowerLimits{};
    std::array<double, 5> upperLimits{};
};
