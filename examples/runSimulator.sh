#!/usr/bin/env bash

# Get the user name and the home directory
USER_NAME=$(whoami)
HOME_DIR="/home/$USER_NAME"

# Set the paths for the car_ws and the path-following-datasets
CAR_WS_PATH="$HOME_DIR/path-following-datasets/examples/car_ws"
DATASET_PATH="$HOME_DIR/path-following-datasets/path_datasets"

xdotool windowsize $(xdotool getactivewindow) 100% 100%

tmux kill-session -t 'simrun'
sleep 1

tmux new-session -d -s 'simrun'  # Added closing quote

tmux rename-window 'Simulator Running Example'
tmux set -g mouse on

tmux split-window -v
tmux select-pane -D
tmux split-window -h
tmux select-pane -D
tmux split-window -v
tmux select-pane -D
tmux select-pane -t 0

tmux send "conda deactivate" C-m
tmux send "cd \"$CAR_WS_PATH\"" C-m
tmux send "source \"$CAR_WS_PATH/devel/setup.bash\"" C-m
tmux send "roslaunch rbcar_sim_bringup rbcar_complete_rl.launch" C-m

tmux select-pane -t 1
tmux send "conda deactivate" C-m
tmux send "cd \"$CAR_WS_PATH\"" C-m
tmux send "source \"$CAR_WS_PATH/devel/setup.bash\"" C-m
tmux send "sleep 3" C-m
tmux send "roslaunch rbcar_control rbcar_control.launch" C-m

tmux select-pane -t 2
tmux send "conda activate mpc-gen" C-m
tmux send "export CAR_WS_PATH=\"$CAR_WS_PATH\"" C-m
tmux send "export DATASET_PATH=\"$DATASET_PATH\"" C-m
tmux send "cd \"$CAR_WS_PATH/src/MPCSimulationRunner/scripts/\"" C-m
tmux send "source \"$CAR_WS_PATH/devel/setup.bash\"" C-m
tmux send "sleep 5" C-m
tmux send "python RunandGatherResults.py" C-m

# Set the environment variables (for car_ws and path_datasets)
tmux select-pane -t 3
tmux send "export CAR_WS_PATH=\"$CAR_WS_PATH\"" C-m
tmux send "export DATASET_PATH=\"$DATASET_PATH\"" C-m
# Check if the environment variables are set correctly
tmux send "echo \"CAR_WS_PATH: $CAR_WS_PATH\"" C-m
tmux send "echo \"DATASET_PATH: $DATASET_PATH\"" C-m

# Optional: Set tiled layout after creating the panes
tmux select-layout tiled

# Optional: Attach to the session in the background
tmux attach-session -d