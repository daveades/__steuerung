cmake_minimum_required(VERSION 3.10)
project(dofbot_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simulated_arm src/main.cpp)
target_compile_features(simulated_arm PRIVATE cxx_std_20)
target_compile_options(simulated_arm PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(simulated_arm PRIVATE rclcpp::rclcpp sensor_msgs::sensor_msgs std_msgs::std_msgs)
install(TARGETS simulated_arm DESTINATION lib/${PROJECT_NAME})

ament_package()
