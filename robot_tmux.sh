#!/bin/bash

SESSION="robot"

# Start a new tmux session
tmux new-session -d -s $SESSION

# Window 1: MAVROS
tmux rename-window -t $SESSION:0 'mavros'
tmux send-keys -t $SESSION:0 'source ~/gps_ws/install/setup.bash && source /opt/ros/humble/setup.bash && ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0' C-m

# Window 2: Realsense
tmux new-window -t $SESSION:1 -n 'realsense'
tmux send-keys -t $SESSION:1 'source ~/gps_ws/install/setup.bash && source /opt/ros/humble/setup.bash && ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30' C-m

# Window 3: Custom Node
tmux new-window -t $SESSION:2 -n 'my_node'
tmux send-keys -t $SESSION:2 'source ~/gps_ws/install/setup.bash && source /opt/ros/humble/setup.bash && ros2 launch realsense_node nodes_launch.py' C-m

# Window 4: Monitor
tmux new-window -t $SESSION:3 -n 'monitor'
tmux send-keys -t $SESSION:3 'jtop' C-m

# Optional: Attach the session if run manually
tmux attach -t $SESSION
