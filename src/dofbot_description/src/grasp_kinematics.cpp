#include <dofbot_description/grasp_kinematics.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

GraspKinematics::GraspKinematics(const std::string &robotDescription)
{
    KDL::Tree tree;
    if (!model.initString(robotDescription) || !kdl_parser::treeFromUrdfModel(model, tree) ||
        !tree.getChain("arm_link5", "rlink2", rightChain) ||
        !tree.getChain("arm_link5", "llink2", leftChain))
    {
        throw std::invalid_argument("The URDF must contain both finger chains from arm_link5");
    }
    const auto referenceJoint = model.getJoint("grasp_frame_joint");
    if (!referenceJoint || referenceJoint->type != urdf::Joint::FIXED ||
        referenceJoint->parent_link_name != "arm_link5" ||
        referenceJoint->child_link_name != "grasp_frame")
    {
        throw std::invalid_argument("The URDF must define a fixed grasp_frame_joint from arm_link5 to grasp_frame");
    }
    const auto &origin = referenceJoint->parent_to_joint_origin_transform;
    const KDL::Vector closedReference(origin.position.x, origin.position.y, origin.position.z);
    rightContact = finger_frame(rightChain, 0.0).Inverse() * closedReference;
    leftContact = finger_frame(leftChain, 0.0).Inverse() * closedReference;
    contact_position(0.0);
}

KDL::Frame GraspKinematics::finger_frame(const KDL::Chain &chain, double gripAngle) const
{
    KDL::JntArray angles(chain.getNrOfJoints());
    std::size_t angleIndex = 0;
    for (const auto &segment : chain.segments)
    {
        if (segment.getJoint().getType() == KDL::Joint::Fixed)
        {
            continue;
        }
        const auto jointName = segment.getJoint().getName();
        const auto joint = model.getJoint(jointName);
        if (jointName == "grip_joint")
        {
            angles(angleIndex++) = gripAngle;
        }
        else if (joint && joint->mimic && joint->mimic->joint_name == "grip_joint")
        {
            angles(angleIndex++) = joint->mimic->multiplier * gripAngle + joint->mimic->offset;
        }
        else
        {
            throw std::invalid_argument("Finger joints must be grip_joint or mimic grip_joint");
        }
    }
    KDL::ChainFkSolverPos_recursive solver(chain);
    KDL::Frame frame;
    if (solver.JntToCart(angles, frame) < 0)
    {
        throw std::runtime_error("Could not calculate the finger transform");
    }
    return frame;
}

std::array<double, 3> GraspKinematics::contact_position(double gripAngle) const
{
    if (!std::isfinite(gripAngle))
    {
        throw std::invalid_argument("Gripper position must be finite");
    }
    const auto midpoint = (finger_frame(rightChain, gripAngle) * rightContact +
        finger_frame(leftChain, gripAngle) * leftContact) * 0.5;
    const std::array<double, 3> position{midpoint.x(), midpoint.y(), midpoint.z()};
    if (!std::all_of(position.begin(), position.end(), [](double value) { return std::isfinite(value); }))
    {
        throw std::runtime_error("Could not calculate a finite gripper contact position");
    }
    return position;
}
