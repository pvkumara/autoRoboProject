#!/bin/bash
# Run from WSL: SSH into Jetson, enter Docker, start all components.
# Usage: bash ~/autoRoboProject/scripts/connect_and_start.sh
#
# Optional: pass the Jetson IP as argument (default: 172.26.126.19)

JETSON_IP=${1:-172.26.126.19}
JETSON_USER=hrishibot
CONTAINER=isaac_ros_dev-aarch64-container
SCRIPTS=/workspaces/isaac_ros-dev/autoRoboProject/scripts

echo "Connecting to $JETSON_USER@$JETSON_IP with X11 forwarding..."

# -X enables X11 forwarding so GUI tools (rqt_image_view, visualizer) work
# DISPLAY is passed through to the container via into_container.sh
ssh -X -t "$JETSON_USER@$JETSON_IP" \
    "docker start $CONTAINER 2>/dev/null; \
     docker exec -it -u admin $CONTAINER bash -c \
       'cd /workspaces/isaac_ros-dev/autoRoboProject && git pull --quiet && bash $SCRIPTS/start_all.sh'"
