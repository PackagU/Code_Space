cmake_minimum_required(VERSION 3.8)
project(common_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY urdf launch worlds DESTINATION share/${PROJECT_NAME}/)

ament_package()
