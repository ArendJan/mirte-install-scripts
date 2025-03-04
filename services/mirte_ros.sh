#!/bin/bash

# by default use minimal launch file, but allow to override to minimal_master
LAUNCH_FILE="${1:-minimal}"

# If the robot user wants to add their own config:
source /home/mirte/.profile
source /home/mirte/mirte_ws/install/setup.bash

# .bashrc wont be sourced in this file, as its not an interactive shell.
ros2 run rmw_zenoh_cpp rmw_zenohd &
ros2 launch mirte_bringup $LAUNCH_FILE.launch.py
