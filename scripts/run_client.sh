#!/bin/bash
# Run inside Docker: starts the object tracker action client
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run object_tracker_client object_tracker_client
