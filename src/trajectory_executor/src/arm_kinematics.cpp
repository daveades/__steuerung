#include "arm_kinematics.hpp"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

ArmKinematics::ArmKinematics(const std::string &robotDescription)
{
    urdf::Model model;
    KDL::Tree tree;
    if (!model.initString(robotDescription) || !kdl_parser::treeFromUrdfModel(model, tree) ||
        !tree.getChain("base_link", "arm_link5", chain) || chain.getNrOfJoints() != 5)
    {
        throw std::invalid_argument("The URDF must contain the five-joint chain from base_link to arm_link5");
    }
    std::size_t jointIndex = 0;
    for (const auto &segment : chain.segments)
    {
        const auto &joint = segment.getJoint();
        if (joint.getType() == KDL::Joint::Fixed)
        {
            continue;
        }
        const auto jointName = "arm_joint" + std::to_string(jointIndex + 1);
        const auto modelJoint = model.getJoint(jointName);
        if (joint.getName() != jointName || !modelJoint || modelJoint->type != urdf::Joint::REVOLUTE ||
            modelJoint->mimic || !modelJoint->limits ||
            !std::isfinite(modelJoint->limits->lower) || !std::isfinite(modelJoint->limits->upper) ||
            modelJoint->limits->lower >= modelJoint->limits->upper)
        {
            throw std::invalid_argument("The arm chain must contain arm_joint1 through arm_joint5 with finite limits");
        }
        lowerLimits[jointIndex] = modelJoint->limits->lower;
        upperLimits[jointIndex] = modelJoint->limits->upper;
        ++jointIndex;
    }
}

KDL::Frame ArmKinematics::wrist_frame(const std::array<double, 5> &jointPositions) const
{
    KDL::JntArray angles(5);
    for (std::size_t jointIndex = 0; jointIndex < jointPositions.size(); ++jointIndex)
    {
        if (!std::isfinite(jointPositions[jointIndex]) || jointPositions[jointIndex] < lowerLimits[jointIndex] ||
            jointPositions[jointIndex] > upperLimits[jointIndex])
        {
            throw std::invalid_argument("Arm angles must be finite and inside the URDF joint limits");
        }
        angles(jointIndex) = jointPositions[jointIndex];
    }
    KDL::ChainFkSolverPos_recursive forwardSolver(chain);
    KDL::Frame frame;
    if (forwardSolver.JntToCart(angles, frame) < 0 ||
        !std::isfinite(frame.p.x()) || !std::isfinite(frame.p.y()) || !std::isfinite(frame.p.z()))
    {
        throw std::runtime_error("Could not calculate a finite wrist position");
    }
    return frame;
}

std::array<double, 3> ArmKinematics::wrist_position(const std::array<double, 5> &jointPositions) const
{
    const auto frame = wrist_frame(jointPositions);
    return {frame.p.x(), frame.p.y(), frame.p.z()};
}

std::array<double, 3> ArmKinematics::target_position(const std::array<double, 5> &jointPositions,
    const std::array<double, 3> &toolOffset) const
{
    if (!std::all_of(toolOffset.begin(), toolOffset.end(),
        [](double value) { return std::isfinite(value); }))
    {
        throw std::invalid_argument("The tool offset must contain three finite coordinates in metres");
    }
    const auto point = wrist_frame(jointPositions) * KDL::Vector(toolOffset[0], toolOffset[1], toolOffset[2]);
    return {point.x(), point.y(), point.z()};
}

ArmSolution ArmKinematics::solve_position(const std::array<double, 5> &currentPositions,
    const std::array<double, 3> &targetPosition, const std::array<double, 3> &toolOffset,
    bool downwardApproach, const std::optional<std::array<double, 3>> &targetDirection) const
{
    target_position(currentPositions, toolOffset);
    if (downwardApproach && targetDirection)
    {
        throw std::invalid_argument("Choose downward or a fixed tool direction, not both");
    }
    const bool constrainedDirection = downwardApproach || targetDirection.has_value();
    const auto requestedDirection = targetDirection.value_or(std::array<double, 3>{0.0, 0.0, -1.0});
    const auto directionVector = normalized_direction(requestedDirection);
    if (!std::all_of(targetPosition.begin(), targetPosition.end(),
        [](double value) { return std::isfinite(value); }))
    {
        throw std::invalid_argument("The XYZ target must contain three finite coordinates in metres");
    }
    Eigen::Matrix<double, 6, 1> weights;
    weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    KDL::Chain solveChain = chain;
    if (constrainedDirection ||
        std::any_of(toolOffset.begin(), toolOffset.end(), [](double value) { return value != 0.0; }))
    {
        solveChain = KDL::Chain{};
        for (const auto &segment : chain.segments)
        {
            if (segment.getJoint().getName() == "arm_joint5")
            {
                solveChain.addSegment(KDL::Segment(segment.getName(), KDL::Joint(KDL::Joint::Fixed),
                    segment.pose(currentPositions[4])));
            }
            else
            {
                solveChain.addSegment(segment);
            }
        }
        solveChain.addSegment(KDL::Segment("contact_target", KDL::Joint(KDL::Joint::Fixed),
            KDL::Frame(KDL::Vector(toolOffset[0], toolOffset[1], toolOffset[2]))));
    }
    KDL::ChainIkSolverPos_LMA inverseSolver(solveChain, weights, 1e-6, 500);
    const KDL::Frame targetFrame(KDL::Vector(targetPosition[0], targetPosition[1], targetPosition[2]));
    constexpr std::array<std::array<double, 2>, 7> seedOffsets{{
        {0.0, 0.0}, {0.15, -0.3}, {-0.15, 0.3}, {0.35, -0.7},
        {-0.35, 0.7}, {0.7, -1.2}, {-0.7, 1.2}}};
    ArmSolution bestSolution{};
    double bestDistance = std::numeric_limits<double>::infinity();
    for (const auto &offset : seedOffsets)
    {
        KDL::JntArray seed(solveChain.getNrOfJoints());
        KDL::JntArray solved(solveChain.getNrOfJoints());
        for (std::size_t jointIndex = 0; jointIndex < solveChain.getNrOfJoints(); ++jointIndex)
        {
            seed(jointIndex) = currentPositions[jointIndex];
        }
        seed(1) = std::clamp(seed(1) + offset[0], lowerLimits[1], upperLimits[1]);
        seed(2) = std::clamp(seed(2) + offset[1], lowerLimits[2], upperLimits[2]);
        if (constrainedDirection ? !solve_direction(solveChain, seed, targetFrame, directionVector, solved) :
            inverseSolver.CartToJnt(seed, targetFrame, solved) < 0)
        {
            continue;
        }
        ArmSolution candidate{};
        double distance = 0.0;
        bool valid = true;
        for (std::size_t jointIndex = 0; jointIndex < currentPositions.size(); ++jointIndex)
        {
            const double angle = jointIndex == 4 ? currentPositions[jointIndex] :
                currentPositions[jointIndex] + std::remainder(
                    solved(jointIndex) - currentPositions[jointIndex], 2.0 * std::numbers::pi);
            if (!std::isfinite(angle) || angle < lowerLimits[jointIndex] || angle > upperLimits[jointIndex])
            {
                valid = false;
                break;
            }
            candidate.jointPositions[jointIndex] = angle;
            distance += (angle - currentPositions[jointIndex]) * (angle - currentPositions[jointIndex]);
        }
        if (!valid)
        {
            continue;
        }
        candidate.wristPosition = wrist_position(candidate.jointPositions);
        candidate.targetPosition = target_position(candidate.jointPositions, toolOffset);
        candidate.errorMetres = std::hypot(candidate.targetPosition[0] - targetPosition[0],
            candidate.targetPosition[1] - targetPosition[1], candidate.targetPosition[2] - targetPosition[2]);
        candidate.downwardErrorDegrees = downwardApproach ? downward_error_degrees(candidate.jointPositions) : 0.0;
        candidate.directionErrorDegrees = constrainedDirection ?
            direction_error_degrees(candidate.jointPositions, requestedDirection) : 0.0;
        if (candidate.errorMetres <= 0.001 && distance < bestDistance &&
            (!constrainedDirection || candidate.directionErrorDegrees <= directionToleranceDegrees))
        {
            bestSolution = candidate;
            bestDistance = distance;
        }
    }
    if (!std::isfinite(bestDistance))
    {
        throw std::runtime_error(downwardApproach ?
            "No valid downward IK solution found within the joint limits, 1 mm position tolerance and 1 degree direction tolerance" :
            targetDirection ?
            "No valid fixed-direction IK solution found within the joint limits, 1 mm position tolerance and 1 degree direction tolerance" :
            "No valid IK solution found within the joint limits and 1 mm position tolerance");
    }
    return bestSolution;
}

std::array<double, 3> ArmKinematics::tool_direction(const std::array<double, 5> &jointPositions) const
{
    const auto direction = wrist_frame(jointPositions).M.UnitZ();
    return {direction.x(), direction.y(), direction.z()};
}

double ArmKinematics::downward_error_degrees(const std::array<double, 5> &jointPositions) const
{
    return direction_error_degrees(jointPositions, {0.0, 0.0, -1.0});
}

KDL::Vector ArmKinematics::normalized_direction(const std::array<double, 3> &direction)
{
    const double length = std::hypot(direction[0], direction[1], direction[2]);
    if (!std::all_of(direction.begin(), direction.end(), [](double value) { return std::isfinite(value); }) ||
        !std::isfinite(length) || length < 1e-12)
    {
        throw std::invalid_argument("Tool direction must contain three finite coordinates and have nonzero length");
    }
    return {direction[0] / length, direction[1] / length, direction[2] / length};
}

double ArmKinematics::direction_error_degrees(const std::array<double, 5> &jointPositions,
    const std::array<double, 3> &targetDirection) const
{
    const auto requested = normalized_direction(targetDirection);
    const auto direction = tool_direction(jointPositions);
    const double alignment = direction[0] * requested.x() + direction[1] * requested.y() +
        direction[2] * requested.z();
    return std::acos(std::clamp(alignment, -1.0, 1.0)) * 180.0 / std::numbers::pi;
}

bool ArmKinematics::solve_direction(const KDL::Chain &solveChain, const KDL::JntArray &seed,
    const KDL::Frame &targetFrame, const KDL::Vector &targetDirection, KDL::JntArray &solved) const
{
    KDL::ChainFkSolverPos_recursive forwardSolver(solveChain);
    KDL::ChainJntToJacSolver jacobianSolver(solveChain);
    KDL::Jacobian jacobian(4);
    KDL::JntArray angles = seed;
    const auto residual = [&targetFrame, &targetDirection](const KDL::Frame &frame) {
        const auto direction = frame.M.UnitZ();
        Eigen::Matrix<double, 6, 1> error;
        error << targetFrame.p.x() - frame.p.x(), targetFrame.p.y() - frame.p.y(),
            targetFrame.p.z() - frame.p.z(), 0.1 * (targetDirection.x() - direction.x()),
            0.1 * (targetDirection.y() - direction.y()), 0.1 * (targetDirection.z() - direction.z());
        return error;
    };
    double damping = 1e-4;
    for (int iteration = 0; iteration < 500; ++iteration)
    {
        KDL::Frame frame;
        if (forwardSolver.JntToCart(angles, frame) < 0 || jacobianSolver.JntToJac(angles, jacobian) < 0)
        {
            return false;
        }
        const auto error = residual(frame);
        if (!error.allFinite())
        {
            return false;
        }
        const auto axis = frame.M.UnitZ();
        const double directionError = std::acos(std::clamp(KDL::dot(axis, targetDirection), -1.0, 1.0)) *
            180.0 / std::numbers::pi;
        if (error.head<3>().norm() <= 0.0001 && directionError <= directionToleranceDegrees)
        {
            break;
        }
        const Eigen::Vector3d direction(axis.x(), axis.y(), axis.z());
        Eigen::Matrix<double, 6, 4> derivative;
        derivative.topRows<3>() = jacobian.data.topRows<3>();
        for (std::size_t jointIndex = 0; jointIndex < 4; ++jointIndex)
        {
            const Eigen::Vector3d angular = jacobian.data.bottomRows<3>().col(jointIndex);
            derivative.bottomRows<3>().col(jointIndex) = 0.1 * angular.cross(direction);
        }
        Eigen::Vector4d step = (derivative.transpose() * derivative +
            damping * Eigen::Matrix4d::Identity()).ldlt().solve(derivative.transpose() * error);
        if (!step.allFinite())
        {
            return false;
        }
        const double largestStep = step.cwiseAbs().maxCoeff();
        if (largestStep < 1e-12)
        {
            break;
        }
        step *= std::min(1.0, 0.2 / largestStep);
        KDL::JntArray candidate = angles;
        for (std::size_t jointIndex = 0; jointIndex < 4; ++jointIndex)
        {
            candidate(jointIndex) = std::clamp(angles(jointIndex) + step(jointIndex),
                lowerLimits[jointIndex], upperLimits[jointIndex]);
        }
        KDL::Frame candidateFrame;
        if (forwardSolver.JntToCart(candidate, candidateFrame) < 0)
        {
            return false;
        }
        if (residual(candidateFrame).squaredNorm() < error.squaredNorm())
        {
            angles = candidate;
            damping = std::max(damping * 0.5, 1e-9);
        }
        else
        {
            damping = std::min(damping * 10.0, 1e6);
        }
    }
    solved = angles;
    return true;
}
