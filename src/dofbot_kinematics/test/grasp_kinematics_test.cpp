#include <dofbot_kinematics/grasp_kinematics.hpp>

#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>

static std::string robot_description()
{
    std::ifstream input(DOFBOT_URDF_PATH);
    if (!input)
    {
        throw std::runtime_error("Could not read the test robot model");
    }
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
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
