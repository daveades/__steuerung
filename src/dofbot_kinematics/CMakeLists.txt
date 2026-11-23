cmake_minimum_required(VERSION 3.10)
project(dofbot_kinematics)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

add_library(grasp_kinematics STATIC src/grasp_kinematics.cpp)
target_include_directories(grasp_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(grasp_kinematics PUBLIC urdf::urdf kdl_parser::kdl_parser orocos-kdl)

add_executable(grasp_reference_publisher src/grasp_reference_publisher.cpp)
target_link_libraries(grasp_reference_publisher PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  tf2_ros::tf2_ros
  grasp_kinematics
)

install(TARGETS grasp_reference_publisher DESTINATION lib/${PROJECT_NAME})
install(TARGETS grasp_kinematics EXPORT grasp_kinematics_targets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(grasp_kinematics_targets)
ament_export_dependencies(urdf kdl_parser orocos_kdl)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(grasp_kinematics_test test/grasp_kinematics_test.cpp)
  target_link_libraries(grasp_kinematics_test grasp_kinematics)
  target_compile_definitions(grasp_kinematics_test PRIVATE
    DOFBOT_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../dofbot_description/urdf/dofbot.urdf")
endif()

ament_package()
