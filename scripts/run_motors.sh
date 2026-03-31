#!/bin/bash
# Run inside Docker: starts the Waveshare motor driver (subscribes to /cmd_vel)
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run waveshare_motor_driver waveshare_motor_driver
