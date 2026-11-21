cmake_minimum_required(VERSION 3.10)
project(dofbot_vision)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui calib3d)

add_library(color_detector STATIC src/color_detector.cpp)
target_compile_features(color_detector PUBLIC cxx_std_20)
target_compile_options(color_detector PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(color_detector PUBLIC src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(color_detector PUBLIC ${OpenCV_LIBS})

add_library(pixel_to_robot STATIC src/pixel_to_robot.cpp)
target_compile_features(pixel_to_robot PUBLIC cxx_std_20)
target_compile_options(pixel_to_robot PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(pixel_to_robot PUBLIC src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pixel_to_robot PUBLIC opencv_core opencv_imgproc opencv_calib3d)

add_library(camera_undistortion STATIC src/camera_undistortion.cpp)
target_compile_features(camera_undistortion PUBLIC cxx_std_20)
target_compile_options(camera_undistortion PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(camera_undistortion PUBLIC src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_undistortion PUBLIC opencv_core opencv_imgproc opencv_calib3d)

add_library(detector_node STATIC src/detector_node.cpp)
target_compile_options(detector_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(detector_node PUBLIC rclcpp::rclcpp color_detector)

add_library(height_aware_pixel_to_robot STATIC src/height_aware_pixel_to_robot.cpp)
target_compile_features(height_aware_pixel_to_robot PUBLIC cxx_std_20)
target_compile_options(height_aware_pixel_to_robot PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(height_aware_pixel_to_robot PUBLIC pixel_to_robot camera_undistortion)

add_executable(image_detector src/image_detector.cpp)
target_compile_options(image_detector PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(image_detector PRIVATE detector_node pixel_to_robot height_aware_pixel_to_robot)

add_executable(camera_detector src/camera_detector.cpp)
target_compile_options(camera_detector PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(camera_detector PRIVATE detector_node camera_undistortion)

install(TARGETS image_detector camera_detector DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(color_detector_test test/color_detector_test.cpp)
  target_link_libraries(color_detector_test color_detector)
  ament_add_gtest(pixel_to_robot_test test/pixel_to_robot_test.cpp)
  target_link_libraries(pixel_to_robot_test pixel_to_robot)
  ament_add_gtest(camera_undistortion_test test/camera_undistortion_test.cpp)
  target_link_libraries(camera_undistortion_test camera_undistortion)
  ament_add_gtest(height_aware_pixel_to_robot_test test/height_aware_pixel_to_robot_test.cpp)
  target_link_libraries(height_aware_pixel_to_robot_test height_aware_pixel_to_robot)
  ament_add_test(image_detector_integration_test
    COMMAND "$<TARGET_FILE:Python3::Interpreter>"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/image_detector_integration_test.py"
      "$<TARGET_FILE:image_detector>"
      "${CMAKE_CURRENT_SOURCE_DIR}/config/colors.yaml"
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_test(camera_detector_integration_test
    COMMAND "$<TARGET_FILE:Python3::Interpreter>"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/camera_detector_integration_test.py"
      "$<TARGET_FILE:camera_detector>"
      "${CMAKE_CURRENT_SOURCE_DIR}/config/colors.yaml"
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()
