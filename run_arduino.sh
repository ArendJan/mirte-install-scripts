#!/bin/bash
# set -x
#TODO: script should have format ./run.sh build|upload] mcu_type
COMMAND=$1
if [ -z "$COMMAND" ]; then
	echo "Usage: $0 build_[mcu] | upload_[mcu]"
	echo "Example: $0 build_nano"
	echo "MCU types: nano, nano_old, pico, stm32"
	exit 1
fi

MIRTE_SRC_DIR=${MIRTE_SRC_DIR:-/usr/local/src/mirte}
export INSTALL_ARDUINO_ALL=${INSTALL_ARDUINO_ALL:-false}
source $MIRTE_SRC_DIR/settings.sh || true
PICO_BUILD_LOCATION=$MIRTE_SRC_DIR/mirte-telemetrix4rpipico/build/Telemetrix4RpiPico

# Check if ROS is running
ROS_RUNNING=0
systemctl is-active mirte-ros | grep '^active' &>/dev/null # "inactive" would've also been grepped, so check for active only
if [ $? == 0 ]; then
	ROS_RUNNING=1
fi
echo "ROS_RUNNING: $ROS_RUNNING"
PROJECT="mirte-telemetrix4arduino"
# Stop ROS when uploading new code
STOPPED_ROS=false
if [[ $COMMAND == upload* ]] && [[ $ROS_RUNNING == "1" ]]; then # test for any upload... command
	echo "STOPPING ROS"
	sudo service mirte-ros stop || /bin/true
	STOPPED_ROS=true
fi
cd $MIRTE_SRC_DIR/$PROJECT || exit 1

buildpico() {
	if [ "$INSTALL_ARDUINO_ALL" != true ]; then
		echo "Using prebuilt uf2, set INSTALL_ARDUINO_ALL to true to build from source"
		return
	fi
	cd $MIRTE_SRC_DIR/mirte-telemetrix4rpipico || exit 1
	# shellcheck disable=SC2164
	mkdir -p build && cd build
	cmake .. -DCMAKE_BUILD_TYPE=Debug
	make
}

upload_pico_uart() {
	sleep 5                             # after ros stopping, pico will reboot bc of watchdog, this takes a few seconds.
	for port in /dev/serial/by-id/*; do # these are only the usb serial ports, not all the other uart ports.
		port=$(realpath $port)
		# send reboot command
		stty 115200 -F $port
		echo -ne '\x01\x26' >$port # 1 byte message, message id 0x26==reset_to_bootloader
		sleep 3                    # wait for the pico to reboot
		# try to upload
		ERR=false
		pico_py_serial_flasher $port $PICO_BUILD_LOCATION.elf || ERR=true
		if $ERR; then
			echo "Failed to upload to Pico using pico_py_serial_flash port $port"
		else
			echo "Successfully uploaded to Pico using pico_py_serial_flash port $port"
			return 0
		fi
	done
	return 1
}

# Different build scripts
if [[ $COMMAND == build* ]]; then
	if test "$COMMAND" == "build"; then
		echo "Building all versions, this will take a while and might need internet connection for tools"
		pio run
	elif test "$COMMAND" == "build_nano"; then
		pio run -e nanoatmega328new
	elif test "$COMMAND" == "build_nano_old"; then
		echo "Building non-default mcu, might need internet connection for tools"
		pio run -e nanoatmega328
	elif test "$COMMAND" == "build_pico"; then
		buildpico
	else
		echo "Unknown build command $COMMAND"
		exit 1
	fi
elif [[ $COMMAND == upload* ]]; then
	# Different upload scripts
	if test "$COMMAND" == "upload" || test "$COMMAND" == "upload_stm32"; then
		echo "Uploading to non-default mcu, might need internet connection for tools"
		pio run -e robotdyn_blackpill_f303cc -t upload
	elif test "$COMMAND" == "upload_nano"; then
		pio run -e nanoatmega328new -t upload
	elif test "$COMMAND" == "upload_nano_old"; then
		echo "Uploading to non-default mcu, might need internet connection for tools"
		pio run -e nanoatmega328 -t upload
	elif test "$1" == "upload_pico"; then
		buildpico
		# This will always upload telemetrix4rpipico.uf2, so no need to pass a file
		ERR=false
		sudo picotool load -f $PICO_BUILD_LOCATION.uf2 || ERR=true
		sleep 2
		sudo picotool verify -f $PICO_BUILD_LOCATION.uf2 && ERR=false || ERR=true # if verifying is okay, then it's also good
		sleep 2
		# if lsusb has pico boot, then run reboot
		if lsusb | grep -q "Raspberry Pi RP2 Boot"; then
			sudo picotool reboot
		fi

		if $ERR; then
			echo "Failed to upload to Pico using picotool, trying using uart"
			upload_pico_uart
			FAILED=$?
			if [ $FAILED -eq 1 ]; then
				echo "Failed to upload using uart."
				echo "Please check the connection and try again"
				echo "Or unplug the Pico, press the BOOTSEL button and plug it in again"
				exit 1
			fi
		fi
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
