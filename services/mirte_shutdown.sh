#!/bin/bash
set -xe
# Don't shutdown if only stopping the service

if ! systemctl list-jobs | grep -q -E 'shutdown.target.*start'; then
	echo "shutdown target not active"
	exit
fi

touch /home/mirte/.shutdown
source /home/mirte/mirte_ws/install/setup.bash
# TODO: does not work if ros is not running

ros2 service call /io/oled/oled/set_text mirte_msgs/srv/SetOLEDText "{ text: 'Shutting down...'}"
sleep 2
