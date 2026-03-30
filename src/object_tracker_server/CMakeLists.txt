cmake_minimum_required(VERSION 3.8)
project(object_tracker_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(object_tracker_interfaces REQUIRED)

add_executable(object_tracker_server
  src/object_tracker_server.cpp
)

ament_target_dependencies(object_tracker_server
  rclcpp
  rclcpp_action
  vision_msgs
  sensor_msgs
  geometry_msgs
  object_tracker_interfaces
)

install(TARGETS object_tracker_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
