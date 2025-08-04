

Following are the required commands


ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0
ros2 launch realsense_node nodes_launch.py


#it might throw hardware errror when it initializes safe to ignore
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30


#for testing purposes use this. The code will only execute when it recieves a valid wp_seq 
ros2 topic pub /mavros/mission/reached mavros_msgs/msg/WaypointReached "{wp_seq: 1}"


Operating tmux: 

when the bot turns on, on ssh-ing the htop window would be visible 
to check logging hit ctrl-b then w to open options to view different processes

#use to force kill the tmux window 
tmux kill-server


source ~/gps_ws/install/setup.bash

source /opt/ros/humble/setup.bash
