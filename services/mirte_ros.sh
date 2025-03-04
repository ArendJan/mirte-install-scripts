#!/bin/bash

# by default use minimal launch file, but allow to override to minimal_master
LAUNCH_FILE="${1:-minimal}"
source /home/mirte/mirte_ws/install/setup.bash

# Not needed anymore, since we use the ROS_DOMAIN_ID
# ip=$(hostname -I | awk '{print $1}') #just get the first ip addr
# if [ "$(echo $ip | wc -w)" -ne 1 ]; then
# 	# happens at boot, when the network is not yet up
# 	echo "multiple or none, wont use the ip from hostname:"
# 	hostname -I
# else
# 	export ROS_IP="$ip"
# 	export ROS_MASTER_URI="http://$ip:11311/"
# 	echo "ROS_IP=$ROS_IP"
# 	echo "ROS_MASTER_URI=$ROS_MASTER_URI"
# fi
# If the robot user wants to add their own config:
export ROS_LOG_DIR=/tmp/ros
mkdir -p $ROS_LOG_DIR
ros2 run rmw_zenoh_cpp rmw_zenohd &
source /home/mirte/.bashrc
ros2 launch mirte_bringup $LAUNCH_FILE.launch.py
