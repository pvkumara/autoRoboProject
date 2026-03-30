cmake_minimum_required(VERSION 3.8)
project(waveshare_motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(waveshare_motor_driver
  src/waveshare_motor_driver.cpp
)

ament_target_dependencies(waveshare_motor_driver
  rclcpp
  geometry_msgs
)

install(TARGETS waveshare_motor_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
