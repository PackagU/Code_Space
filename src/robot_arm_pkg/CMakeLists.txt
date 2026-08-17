cmake_minimum_required(VERSION 3.8)
project(robot_arm_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  robot_arm_pkg/arm_sequence_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_sequence
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
endif()

ament_package()
