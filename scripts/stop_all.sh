#!/bin/bash
# Gracefully stop all tracker components.
# Sends Ctrl+C to the motors window first so the motor driver destructor
# runs and sends a zero-velocity command to the wheels before shutting down.
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null || true

SESSION=tracker

if ! tmux has-session -t $SESSION 2>/dev/null; then
    echo "No '$SESSION' tmux session running."
    exit 0
fi

echo "Stopping motors gracefully..."
# Send Ctrl+C to motors window and wait for the stop command to reach the ESP32
tmux send-keys -t $SESSION:motors C-c
sleep 1

# Also publish a zero-velocity just in case
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" 2>/dev/null || true
sleep 0.5

echo "Killing tmux session..."
tmux kill-session -t $SESSION
echo "Done."
