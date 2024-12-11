#!/bin/bash
#TODO: script should have format ./run.sh build|upload] mcu_type
MIRTE_SRC_DIR=/usr/local/src/mirte
# Check if ROS is running
ROS_RUNNING=$(systemctl is-active mirte-ros || /bin/true)
COMMAND=$1
PROJECT="mirte-telemetrix4arduino"
# Stop ROS when uploading new code
STOPPED_ROS=false
if [[ $COMMAND == upload* ]] && [[ $ROS_RUNNING == "1" ]]; then # test for any upload... command
	echo "STOPPING ROS"
	sudo service mirte-ros stop || /bin/true
	STOPPED_ROS=true
fi
cd $MIRTE_SRC_DIR/$PROJECT || exit 1

# Different build scripts
if [[ $COMMAND == build* ]]; then
	if test "$COMMAND" == "build"; then
		pio run
	elif test "$COMMAND" == "build_nano"; then
		pio run -e nanoatmega328new
		arduino-cli -v compile --fqbn arduino:avr:nano:cpu=atmega328 /home/mirte/arduino_project/$PROJECT
	elif test "$COMMAND" == "build_nano_old"; then
		pio run -e nanoatmega328
		arduino-cli -v compile --fqbn arduino:avr:nano:cpu=atmega328old /home/mirte/arduino_project/$PROJECT
	elif test "$COMMAND" == "build_pico"; then
		cd $MIRTE_SRC_DIR/mirte-telemetrix4rpipico || exit 1
		# shellcheck disable=SC2164
		cd build || mkdir build && cd build
		cmake ..
		make
	else
		echo "Unknown build command $COMMAND"
		exit 1
	fi
elif [[ $COMMAND == upload* ]]; then
	# Different upload scripts
	if test "$COMMAND" == "upload" || test "$COMMAND" == "upload_stm32"; then
		pio run -e robotdyn_blackpill_f303cc -t upload
	elif test "$COMMAND" == "upload_nano"; then
		pio run -e nanoatmega328new -t upload
	elif test "$COMMAND" == "upload_nano_old"; then
		pio run -e nanoatmega328 -t upload
	elif test "$1" == "upload_pico"; then
		# This will always upload telemetrix4rpipico.uf2, so no need to pass a file
		sudo picotool load -f $MIRTE_SRC_DIR/mirte-telemetrix4rpipico/Telemetrix4RpiPico.uf2
		retVal=$?
		if [ $retVal -ne 0 ]; then
			echo "Failed to upload to Pico"
			echo "Please check the connection and try again"
			echo "Or unplug the Pico, press the BOOTSEL button and plug it in again"
			exit 1
		fi
		sudo picotool reboot # just to make sure, sometimes it does not reboot automatically
	else
		echo "Unknown upload command $COMMAND"
		exit 1
	fi
else
	echo "Unknown command $COMMAND"
	exit 1
fi

# Start ROS again
if $STOPPED_ROS; then
	sudo service mirte-ros start
	echo "STARTING ROS"
else
	echo "NOT STARTING ROS"
	echo "Start it yourself with 'sudo service mirte-ros start'"
fi
