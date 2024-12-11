#!/bin/bash
set -x
. /opt/ros/humble/setup.bash
. /home/mirte/mirte_ws/install/setup.bash
SECONDS=0
WARN_LVL=0
while true; do
	OK=false

	# echo "topics"
	percentage=$(
		(ros2 topic echo /mirte/power/power_watcher --once | grep percentage | tail -1) &
		pid=$!
		(sleep 5 && kill -9 $pid) 2>/dev/null &
		watcher=$!
		wait $pid 2>/dev/null && pkill -HUP -P $watcher
	)
	# echo $percentage
	# echo $( echo $percentage | wc -c)
	if [ "$(echo $percentage | wc -c)" -gt 1 ]; then
		# echo "percentage"
		percentage=$(echo "$percentage" | awk '{print $NF}')
		# echo $percentage
		if (($(echo "$percentage > 0.1" | bc -l))); then
			OK=true
		fi
	fi

	if $OK; then
		# echo "latest percentage: "
		# echo $percentage
		SECONDS=0 # update time since last ok
		WARN_LVL=0
	fi
	if [ $SECONDS -gt 300 ] && [ $WARN_LVL -eq 0 ]; then
		wall "No ROS for longer than 5min"
		WARN_LVL=1
	fi
	if [ $SECONDS -gt 600 ] && [ $WARN_LVL -eq 1 ]; then
		wall "No ROS for longer than 10min, shutting down in 5min"
		WARN_LVL=2
	fi
	if [ $SECONDS -gt 900 ] && [ $WARN_LVL -eq 2 ]; then
		wall "shutting down"
		WARN_LVL=3
		date >/home/mirte/.shutdown_power
		sudo shutdown now
	fi
	sleep 10
done
