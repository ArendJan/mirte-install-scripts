#!/bin/bash

# by default use minimal launch file, but allow to override to minimal_master
LAUNCH_FILE="${1:-minimal}"

export ROS_LOG_DIR=/tmp/ros
mkdir -p $ROS_LOG_DIR
# If the robot user wants to add their own config:
source /home/mirte/.profile
# .bashrc wont be sourced in this file, as its not in an interactive shell.

source /home/mirte/mirte_ws/install/setup.bash

ros2 run rmw_zenoh_cpp rmw_zenohd &
ros2 launch mirte_bringup $LAUNCH_FILE.launch.py
