#include <chrono>
#include <cstdint>
#include <functional>
#include <stdexcept>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher() : Node("joint_state")
    {
        rcl_interfaces::msg::ParameterDescriptor intervalDescriptor;
        intervalDescriptor.description = "Publishing interval in milliseconds; set at startup";
        intervalDescriptor.read_only = true;
        const auto publishIntervalMs = this->declare_parameter<std::int64_t>(
            "publish_interval_ms", 5, intervalDescriptor);
        if (publishIntervalMs <= 0)
            throw std::invalid_argument("publish_interval_ms must be greater than zero");

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publishIntervalMs),
            std::bind(&JointStatePublisher::reportJointStates, this));

        RCLCPP_INFO(this->get_logger(), "Publishing joint states every %lld ms",
            static_cast<long long>(publishIntervalMs));
    }

private:
    auto getJointStates()
    {
        /*still empty for now*/
        return (0);
    }

    const double MAX{0.78450};
    const double MIN{-0.78450};

    double currentAngle{MIN};
    int directionSwitch = -1;

    void reportJointStates()
    {
        sensor_msgs::msg::JointState fakeJointData;
        if (currentAngle >= MAX)
        {
            currentAngle = MAX;
            directionSwitch *= -1;
        }
        else if (currentAngle <= MIN)
        {
            currentAngle = MIN;
            directionSwitch *= -1;
        }

        fakeJointData.header.stamp = this->now();
        fakeJointData.name = {"Servo1", "Servo2"};
        fakeJointData.position = {currentAngle, 0.7845};

        publisher_->publish(fakeJointData);

        this->currentAngle += (this->directionSwitch * 0.0005);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<JointStatePublisher>());
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("joint_state"), "%s", error.what());
        rclcpp::shutdown();
        return (1);
    }
    rclcpp::shutdown();

    return (0);
}
