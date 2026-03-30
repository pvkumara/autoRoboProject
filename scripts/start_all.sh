#!/bin/bash
# Run inside Docker: starts all three components in a tmux session.
# Usage: bash /workspaces/isaac_ros-dev/autoRoboProject/scripts/start_all.sh
#
# Pane layout:
#   Window 0 "pipeline" — RealSense + YOLOv8
#   Window 1 "server"   — Action server
#   Window 2 "client"   — Action client
#
# Attach to the session with:  tmux attach -t tracker

SCRIPTS=/workspaces/isaac_ros-dev/autoRoboProject/scripts
SESSION=tracker

# Install tmux if missing
if ! command -v tmux &> /dev/null; then
    echo "tmux not found — installing..."
    sudo apt-get install -y tmux
fi

# Kill any old session with the same name
tmux kill-session -t $SESSION 2>/dev/null

# Window 0: pipeline
tmux new-session  -d -s $SESSION -n pipeline \
    "bash $SCRIPTS/run_pipeline.sh; exec bash"

# Give the pipeline a few seconds to start before launching server
sleep 3

# Window 1: action server
tmux new-window -t $SESSION -n server \
    "bash $SCRIPTS/run_server.sh; exec bash"

# Window 2: action client
tmux new-window -t $SESSION -n client \
    "bash $SCRIPTS/run_client.sh; exec bash"

# Focus the client window by default
tmux select-window -t $SESSION:client

echo ""
echo "All components started in tmux session '$SESSION'."
echo ""
echo "Attach with:          tmux attach -t $SESSION"
echo "Switch windows:       Ctrl+B then 0 / 1 / 2"
echo "Scroll in a pane:     Ctrl+B then [ (q to quit scroll)"
echo "Kill everything:      tmux kill-session -t $SESSION"
echo ""
tmux attach -t $SESSION
