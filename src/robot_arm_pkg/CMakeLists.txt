cmake_minimum_required(VERSION 3.8)
project(robot_arm_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# launch/ 는 아직 비어 있어 git에 없음 — fresh clone 빌드가 깨지지 않게 가드
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
endif()

ament_package()
