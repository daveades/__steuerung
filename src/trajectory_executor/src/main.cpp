#include <functional>
#include <sstream>
#include <cstddef>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class JointTrajectory : public rclcpp::Node
{
public:
    JointTrajectory() : Node("joint_trajectory")
    {
        subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/planned_trajectory", 10,
            std::bind(&JointTrajectory::logJointTrajectory, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&JointTrajectory::startNextTrajectory, this)
        );
    }

private:
    int newTrajectory {0};
    trajectory_msgs::msg::JointTrajectory receivedTrajectory;
    rclcpp::Time startTime;
    size_t nextPointIndex;

    void startNextTrajectory()
    {
        if (!newTrajectory)
            return;
        if (nextPointIndex < receivedTrajectory.points.size())
        {
            if ((this->now() - startTime) >= rclcpp::Duration(receivedTrajectory.points[nextPointIndex].time_from_start))
            {
                RCLCPP_INFO(this->get_logger(), "Point[%zu]", nextPointIndex);
                nextPointIndex += 1;
            }
        }
        else
            newTrajectory = 0;
    }

    void logJointTrajectory(const trajectory_msgs::msg::JointTrajectory &trajectoryPayload)
    {

        if (trajectoryPayload.joint_names.empty() || trajectoryPayload.points.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Either joint_name or trajectory_points is empty");
            return;
        }

        for (size_t j{0}; j < trajectoryPayload.points.size(); j++)
        {
            if (trajectoryPayload.points[j].positions.size() != trajectoryPayload.joint_names.size())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Invalid trajectory data: Recieved %zu names but %zu position at point %zu",
                    trajectoryPayload.joint_names.size(),
                    trajectoryPayload.points[j].positions.size(),
                    j
                );
                return;    
            }

            if (trajectoryPayload.points[j].time_from_start.sec < 0)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Invalid time_from_start recieved at point %zu",
                    j
                );
                return;
            }
            if (j != 0)
            {
                rclcpp::Duration prevTime(trajectoryPayload.points[j-1].time_from_start);
                rclcpp::Duration currTime(trajectoryPayload.points[j].time_from_start); 
                
                if (currTime <= prevTime)
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid trajectory time received");
                    return;
                }
            }
        }

        this->newTrajectory = 1;
        receivedTrajectory = trajectoryPayload;
        startTime = this->now();
        nextPointIndex = 0;

        std::ostringstream strOut;

        strOut << "\"Points\": [";
        for (size_t i{0}; i < trajectoryPayload.points.size(); i++)
        {   
            strOut << "{\"time_from_start\": {"
            << "\"sec\": "
            << trajectoryPayload.points[i].time_from_start.sec
            << ", \"nanosec\": "
            << trajectoryPayload.points[i].time_from_start.nanosec
            << "}"
            << ", \"positions\": {";
            for (size_t k{0}; k < trajectoryPayload.joint_names.size(); k++)
            {
                strOut << "\""
                << trajectoryPayload.joint_names[k]
                << "\""
                << ": "
                << trajectoryPayload.points[i].positions[k];

                if (k != trajectoryPayload.joint_names.size() - 1)
                    strOut << ", ";
            }
            strOut << "}}";
            if (i != trajectoryPayload.points.size() - 1)
                    strOut << ", ";
        }
        strOut << "]";

        RCLCPP_INFO(this->get_logger(), "{%s}", strOut.str().c_str());
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectory>());
    rclcpp::shutdown();
}
