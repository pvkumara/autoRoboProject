#!/bin/bash
# Drop-in replacement for: docker exec -it -u admin isaac_ros_dev-aarch64-container /bin/bash
#
# Automatically forwards X11 display so GUI tools work exactly as the
# Isaac ROS docs describe (rqt_image_view, isaac_ros_yolov8_visualizer.py, etc.)
#
# Usage (on the Jetson, after connecting via: ssh -X hrishibot@<IP>):
#   bash ~/into_container.sh
#
# Then inside the container the docs commands work as-is:
#   ros2 run isaac_ros_yolov8 isaac_ros_yolov8_visualizer.py
#   ros2 run rqt_image_view rqt_image_view /yolov8_processed_image

CONTAINER=isaac_ros_dev-aarch64-container

# Allow the container to connect to the host X server
xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | \
    xauth -f /tmp/.docker.xauth nmerge - 2>/dev/null

docker exec -it -u admin \
    -e DISPLAY="$DISPLAY" \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /tmp/.docker.xauth:/tmp/.docker.xauth \
    "$CONTAINER" /bin/bash
