cmake_minimum_required(VERSION 3.10)
project(dofbot_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

add_library(grasp_kinematics STATIC src/grasp_kinematics.cpp)
target_compile_features(grasp_kinematics PUBLIC cxx_std_20)
target_compile_options(grasp_kinematics PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(grasp_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(grasp_kinematics PUBLIC urdf::urdf kdl_parser::kdl_parser orocos-kdl)

add_executable(grasp_reference_publisher src/grasp_reference_publisher.cpp)
target_compile_features(grasp_reference_publisher PRIVATE cxx_std_20)
target_compile_options(grasp_reference_publisher PRIVATE -Wall -Wextra -Wpedantic)
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

install(DIRECTORY launch meshes rviz urdf DESTINATION share/${PROJECT_NAME})

ament_package()
