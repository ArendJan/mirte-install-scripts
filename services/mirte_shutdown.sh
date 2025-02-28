#!/bin/bash
# set -xe
# Don't shutdown if only stopping the service

if ! systemctl list-jobs | grep -q -E 'shutdown.target.*start'; then
	echo "shutdown target not active"
	exit
fi

source /home/mirte/.bashrc
touch /home/mirte/.shutdown
service=/io/oled/oled/set_text
if [ "$MIRTE_USE_MULTIROBOT" = "true" ]; then
	mirte_space=$(cat /etc/hostname | tr '[:upper:]' '[:lower:]' | tr '-' '_')
	service="/$mirte_space$service"
fi

if [ "$(ros2 service list | grep "$service$")" ]; then
	ros2 service call "$service" mirte_msgs/srv/SetOLEDText "{ text: 'Shutting down...'}"
fi

sleep 2
