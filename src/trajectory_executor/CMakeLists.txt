cmake_minimum_required(VERSION 3.8)
project(trajectory_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dofbot_description REQUIRED)

add_library(arm_kinematics STATIC src/arm_kinematics.cpp)
target_compile_features(arm_kinematics PUBLIC cxx_std_20)
target_include_directories(arm_kinematics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(arm_kinematics PUBLIC
  urdf::urdf
  kdl_parser::kdl_parser
  orocos-kdl
  Eigen3::Eigen
  dofbot_description::grasp_kinematics
)

add_executable(pose_runner src/pose_runner.cpp)
target_compile_features(pose_runner PRIVATE cxx_std_20)
target_link_libraries(pose_runner PRIVATE
  rclcpp::rclcpp
  rcl_interfaces::rcl_interfaces
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  arm_kinematics
)

install(TARGETS pose_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  ament_add_gtest(arm_kinematics_test test/arm_kinematics_test.cpp)
  target_link_libraries(arm_kinematics_test arm_kinematics ament_index_cpp::ament_index_cpp)
  add_test(NAME pose_runner_confirmation_test
    COMMAND ${Python3_EXECUTABLE} -u ${CMAKE_CURRENT_SOURCE_DIR}/test/pose_runner_confirmation_test.py
      $<TARGET_FILE:pose_runner>)
  set_tests_properties(pose_runner_confirmation_test PROPERTIES TIMEOUT 600)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
