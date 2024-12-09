#!/bin/bash

cd Piper_ros_private-ros-noetic && bash can_config.sh

source devel/setup.bash

cd ..

# Start a new tmux session named 'collect_data'
tmux new-session -d -s collect_data

# Split the window into two vertical panes
tmux split-window -h -t collect_data:0.0
tmux split-window -v -t collect_data:0.0
tmux split-window -v -t collect_data:0.2

# Run commands in each pane
# tmux send-keys -t collect_data:0.0 "cd Piper_ros_private-ros-noetic" C-m
# tmux send-keys -t collect_data:0.0 "bash can_config.sh" C-m
tmux send-keys -t collect_data:0.0 "roscore" C-m

sleep 1

# tmux send-keys -t collect_data:0.1 "cd Piper_ros_private-ros-noetic && bash can_config.sh && source devel/setup.bash" C-m
tmux send-keys -t collect_data:0.1 "roslaunch piper start_ms_piper.launch mode:=0 auto_enable:=false" C-m

tmux send-keys -t collect_data:0.2 "roslaunch astra_camera multi_camera.launch" C-m

tmux send-keys -t collect_data:0.3 "rostopic list" C-m
tmux send-keys -t collect_data:0.3 "cd collect_data" C-m
tmux send-keys -t collect_data:0.3 "python collect_data.py --dataset_dir data --task_name cobot_demo --max_timesteps 500 --episode_idx 0"

# Attach to the tmux session
tmux attach-session -t collect_data
