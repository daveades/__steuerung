#include <functional>
#include <memory>
#include <cstddef>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class StateMonitor : public rclcpp::Node
{
public:
    StateMonitor() : Node("state_monitor")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&StateMonitor::logJointStates, this, std::placeholders::_1));
    }

private:
    void logJointStates(const sensor_msgs::msg::JointState &jointPayload) const
    {
        if (jointPayload.name.empty() || jointPayload.position.empty())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Joint Payload is empty: received %zu joint names and %zu positions.",
                jointPayload.name.size(),
                jointPayload.position.size());
            return;
        }
        else if (jointPayload.name.size() != jointPayload.position.size())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Joint payload size mismatch: received %zu joint name(s) but %zu position(s).",
                jointPayload.name.size(),
                jointPayload.position.size());
            return;
        }

        std::ostringstream jointString;
        
        for (std::size_t i{0}; i < jointPayload.position.size(); i++)
        {

            jointString << "\"" << jointPayload.name[i] << "\"" << ": " << jointPayload.position[i];

            if (i != jointPayload.name.size() - 1)
                jointString << ", ";
        }

        RCLCPP_INFO(
            this->get_logger(),

            "{\"timestamp\": \"%d.%09u\", \"joint_states\": {%s}}",
            jointPayload.header.stamp.sec,
            jointPayload.header.stamp.nanosec,
            jointString.str().c_str());
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMonitor>());
    rclcpp::shutdown();

    return (0);
}
