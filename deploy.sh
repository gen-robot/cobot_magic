#!/bin/bash

cd Piper_ros_private-ros-noetic && bash can_config.sh

source devel/setup.bash

cd ..

# Start a new tmux session named 'deploy'
tmux new-session -d -s deploy

# Split the window into two vertical panes
tmux split-window -h -t deploy:0.0
tmux split-window -v -t deploy:0.0
tmux split-window -v -t deploy:0.2

# Run commands in each pane
# tmux send-keys -t deploy:0.0 "cd Piper_ros_private-ros-noetic" C-m
# tmux send-keys -t deploy:0.0 "bash can_config.sh" C-m
tmux send-keys -t deploy:0.0 "roscore" C-m

sleep 1

# tmux send-keys -t deploy:0.1 "cd Piper_ros_private-ros-noetic && bash can_config.sh && source devel/setup.bash" C-m
tmux send-keys -t deploy:0.1 "roslaunch piper start_ms_piper.launch mode:=1 auto_enable:=true" C-m

tmux send-keys -t deploy:0.2 "roslaunch astra_camera multi_camera.launch" C-m

tmux send-keys -t deploy:0.3 "conda activate embodied; cd ~/arm_ws/EmbodiedAgent/scripts; rostopic list" C-m

# Attach to the tmux session
tmux attach-session -t deploy
