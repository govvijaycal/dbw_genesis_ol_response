#!/bin/bash
# REFERENCE: https://gist.github.com/davb5/7233970

# TO SET EACH TIME:
MODE="step" # "step", "ramp", or "sine"
SESSION="ros"
tmux -2 new-session -d -s $SESSION

tmux rename-window "ROS"
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# PANE 0: roscore
tmux select-pane -t 0
tmux send-keys "roscore" C-m

# PANE 1: rosbag
tmux select-pane -t 1
tmux send-keys "roscd dbw_genesis_ol_response/data" C-m
tmux send-keys "rosbag record -a -o spas_$MODE.bag" C-m

# PANE 2: OL demo
tmux select-pane -t 2
tmux send-keys "roscd dbw_genesis_ol_response/scripts" C-m
tmux send-keys "python demo_open_loop_steer.py --mode $MODE" # force user to start this.

# PANE 2: rostopic echo
tmux select-pane -t 3
tmux send-keys "roscd dbw_genesis_ol_response/" C-m
tmux send-keys "rostopic echo /vehicle/steering" # force user to start this.

tmux select-window -t "ROS"
tmux select-pane -t 2

tmux -2 attach-session -t $SESSION
