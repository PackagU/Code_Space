cmake_minimum_required(VERSION 3.8)
project(drive_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# drive_pkg/ 파이썬 모듈 전체를 site-packages로 설치 (노드 간 import용)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  drive_pkg/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_teleop
)

install(PROGRAMS
  drive_pkg/opencr_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME opencr_bridge
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
endif()

ament_package()
