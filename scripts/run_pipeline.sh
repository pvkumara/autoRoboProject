#!/bin/bash
# Run inside Docker: starts the RealSense + YOLOv8 pipeline
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch object_tracker_server pipeline.launch.py
