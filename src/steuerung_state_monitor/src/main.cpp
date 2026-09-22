#include <iostream>
#include "rclcpp/rclcpp.hpp"


class StateMonitor: public rclcpp::Node
{
    public:
        StateMonitor():
            Node("stateMonitor")
        {
            std::cout << "This is my first Node" << std::endl;
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMonitor>());
    rclcpp::shutdown();

    return (0);
}
