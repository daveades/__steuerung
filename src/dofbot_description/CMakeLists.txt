cmake_minimum_required(VERSION 3.10)
project(dofbot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf DESTINATION share/${PROJECT_NAME})

ament_package()
