#!/bin/bash
# Run inside Docker: starts the object tracker action client
source /workspaces/isaac_ros-dev/install/setup.bash

LOG_DIR=/workspaces/isaac_ros-dev/autoRoboProject/logs
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/tracker_run_$(date '+%Y-%m-%d_%H-%M-%S').log"
echo "=== Tracker client started: $(date) ===" | tee "$LOG_FILE"
echo "Logging to: $LOG_FILE"

ros2 run object_tracker_client object_tracker_client 2>&1 | tee -a "$LOG_FILE"

echo "=== Tracker client exited: $(date) ===" | tee -a "$LOG_FILE"
