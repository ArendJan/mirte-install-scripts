#!/bin/bash

# by default use minimal launch file, but allow to override to minimal_master
LAUNCH_FILE="${1:-minimal}"

# If the robot user wants to add their own config:
source /home/mirte/.mirte_settings
mkdir -p $ROS_LOG_DIR

source /home/mirte/mirte_ws/install/setup.bash
# if zenoh is enabled, start the zenoh daemon
if [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ]; then
	# kill ros deamon
	sudo pkill -9 -f ^ros && ros2 daemon stop
	ros2 run rmw_zenoh_cpp rmw_zenohd &
fi

ros2 launch mirte_bringup $LAUNCH_FILE.launch.py
