#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher() : Node("joint_state")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&JointStatePublisher::reportJointStates, this));

        RCLCPP_INFO(this->get_logger(), "Publishing joint states");
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
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();

    return (0);
}