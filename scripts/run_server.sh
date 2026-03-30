#!/bin/bash
# Run inside Docker: starts the object tracker action server
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run object_tracker_server object_tracker_server
