cmake_minimum_required(VERSION 3.8)
project(slam_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  maps
  web
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  slam_pkg/field_web_ui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME field_web_ui
)

ament_package()
