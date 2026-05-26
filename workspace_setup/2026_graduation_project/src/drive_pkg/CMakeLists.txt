cmake_minimum_required(VERSION 3.8)
project(drive_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
endif()

ament_package()
