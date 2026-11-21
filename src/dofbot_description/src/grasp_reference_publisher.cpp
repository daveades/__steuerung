#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <dofbot_description/grasp_kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

class GraspReferencePublisher : public rclcpp::Node
{
public:
    GraspReferencePublisher()
        : Node("grasp_reference_publisher")
    {
        const auto robotDescription = declare_parameter<std::string>("robot_description", "");
        kinematics = std::make_unique<GraspKinematics>(robotDescription);
        broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        feedbackSubscription = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::QoS(10),
            [this](const sensor_msgs::msg::JointState &measurement) { publish_reference(measurement); });
        RCLCPP_INFO(get_logger(), "Grasp contact frame enabled: parent=arm_link5, child=grasp_contact_frame");
    }

private:
    void publish_reference(const sensor_msgs::msg::JointState &measurement)
    {
        const auto grip = std::find(measurement.name.begin(), measurement.name.end(), "grip_joint");
        if (grip == measurement.name.end())
        {
            return;
        }
        if (measurement.position.size() != measurement.name.size() ||
            std::count(measurement.name.begin(), measurement.name.end(), "grip_joint") != 1)
        {
            RCLCPP_WARN(get_logger(), "Grasp contact feedback rejected: joint names and positions must match, with one grip_joint");
            return;
        }
        const auto gripIndex = static_cast<std::size_t>(grip - measurement.name.begin());
        const double gripAngle = measurement.position[gripIndex];
        if (!std::isfinite(gripAngle))
        {
            RCLCPP_WARN(get_logger(), "Grasp contact feedback rejected: grip_joint position must be finite");
            return;
        }
        const auto midpoint = kinematics->contact_position(gripAngle);
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = measurement.header.stamp;
        transform.header.frame_id = "arm_link5";
        transform.child_frame_id = "grasp_contact_frame";
        transform.transform.translation.x = midpoint[0];
        transform.transform.translation.y = midpoint[1];
        transform.transform.translation.z = midpoint[2];
        transform.transform.rotation.w = 1.0;
        broadcaster->sendTransform(transform);
    }

    std::unique_ptr<GraspKinematics> kinematics;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedbackSubscription;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    int exitCode = 0;
    try
    {
        rclcpp::spin(std::make_shared<GraspReferencePublisher>());
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("grasp_reference_publisher"), "%s", error.what());
        exitCode = 1;
    }
    rclcpp::shutdown();
    return exitCode;
}
