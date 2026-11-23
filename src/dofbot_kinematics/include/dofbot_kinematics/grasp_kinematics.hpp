#pragma once

#include <kdl/chain.hpp>
#include <urdf/model.h>

#include <array>
#include <string>

class GraspKinematics
{
public:
    explicit GraspKinematics(const std::string &robotDescription);

    std::array<double, 3> contact_position(double gripAngle) const;

private:
    KDL::Frame finger_frame(const KDL::Chain &chain, double gripAngle) const;

    urdf::Model model;
    KDL::Chain rightChain;
    KDL::Chain leftChain;
    KDL::Vector rightContact;
    KDL::Vector leftContact;
};
