cmake_minimum_required(VERSION 3.8)
project(object_tracker_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(object_tracker_interfaces REQUIRED)

add_executable(object_tracker_client
  src/object_tracker_client.cpp
)

ament_target_dependencies(object_tracker_client
  rclcpp
  rclcpp_action
  object_tracker_interfaces
)

install(TARGETS object_tracker_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
