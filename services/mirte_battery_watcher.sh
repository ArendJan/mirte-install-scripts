#!/bin/bash
set -x
. /home/mirte/.mirte_settings.sh
. /opt/ros/humble/setup.bash
. /home/mirte/mirte_ws/install/setup.bash
SECONDS=0
LAST_SECONDS=0
WARN_LVL=0

# if battery <= 10%, shutdown in 2 min.
# otherwise if longer than 15 min no message, shutdown.

mirte_space=$(cat /etc/hostname | tr '[:upper:]' '[:lower:]' | tr '-' '_')
BATTERY_FILE=/tmp/batteryState
STOP=false
trap 'echo "Stopping mirte_master_check"; kill $ECHO_PID; STOP=true' SIGTERM SIGINT
# ros2 topic echo restarts when the topic appears again, so we can just echo always
# on every check, it will read the file, take the last reading and then clear the file
# if the sensor is off, then the file will be empty and will trigger shutdown after some time.
topic=/io/power/power_watcher
if [ "$MIRTE_USE_MULTIROBOT" = "true" ]; then
	topic=/$mirte_space/io/power/power_watcher
fi
ros2 topic echo $topic sensor_msgs/msg/BatteryState --field percentage >$BATTERY_FILE &
ECHO_PID=$!
while ! systemctl list-jobs | grep -q -E 'shutdown.target.*start' && ! $STOP; do
	OK=false
	# echo "topics"
	percentage=$(
		tail -2 $BATTERY_FILE | head -1
	) || true
	echo $percentage
	true >$BATTERY_FILE
	# percentage=$(echo "$percentage" | tail -1)
	# echo $percentage
	# echo $( echo $percentage | wc -c)
	if [ "$(echo $percentage | wc -c)" -gt 1 ]; then
		# echo "percentage"
		percentage=$(echo "$percentage" | awk '{print $NF}')
		# convert to integer *100/1 to avoid floating point comparison
		percentage=$(echo "scale=0; $percentage * 100/1" | bc)
		# echo $percentage
		if (($(echo "$percentage > 10" | bc -l))); then
			OK=true
		elif (($(echo "$percentage <= 10" | bc -l))); then
			wall "Battery is low, shutting down in 2min"
			date >>/home/mirte/.shutdown_battery
			sudo shutdown +2
			STOP=true
		fi
	fi

	if $OK; then
		# echo "latest percentage: "
		# echo $percentage
		SECONDS=0 # update time since last ok
		LAST_SECONDS=0
		WARN_LVL=0
	fi
	# if there is a time jump and seconds is more than last_seconds+2, reset seconds
	# time jumps happen when the time is updated (on connecting to wifi)
	next_second=$((LAST_SECONDS + 20))
	if [ $SECONDS -gt $next_second ]; then
		echo "time jump"
		SECONDS=0
		LAST_SECONDS=0
		WARN_LVL=0
	fi
	LAST_SECONDS=$SECONDS

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
		date >>/home/mirte/.shutdown_power
		sudo shutdown now
		STOP=true
	fi
	sleep 10
done

kill $ECHO_PID || true
jobs -p
ros2 daemon stop || true # deamon is sometimes started by ros2 topic echo, otherwise the shutdown will get stuck and wait for 30+ seconds
