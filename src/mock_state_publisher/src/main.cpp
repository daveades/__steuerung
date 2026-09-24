#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher() : Node("joint_state_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JointStatePublisher::reportJointStates, this));
    }

private:
    auto getJointStates()
    {
        /*still empty for now*/
        return (0);
    }

    void reportJointStates()
    {
        sensor_msgs::msg::JointState fakeJointData;

        fakeJointData.name = {"Servo1", "Servo2"};
        fakeJointData.position = {90.56, 45};

        RCLCPP_INFO(this->get_logger(), "Publishing joint states");
        publisher_->publish(fakeJointData);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();

    return (0);
}