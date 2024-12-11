#!/bin/bash

#TODO: script should have format ./run.sh build|upload] mcu_type arduino_folder
# with mcu_type and arduino_folder optional

# Check if ROS is running
ROS_RUNNING=$(ps aux | grep -c "[r]osmaster")

# Stop ROS when uploading new code
STOPPED_ROS=false
if [[ $1 == upload* ]] && [[ $ROS_RUNNING == "1" ]]; then # test for any upload... command
	echo "STOPPING ROS"
	sudo service mirte-ros stop || /bin/true
	STOPPED_ROS=true
fi

# Different build scripts
if test "$1" == "build"; then
	arduino-cli -v compile --fqbn STM32:stm32:GenF1:pnum=BLUEPILL_F103C8,upload_method=dfu2Method,xserial=generic,usb=CDCgen,xusb=FS,opt=osstd,rtlib=nano /home/mirte/arduino_project/$2
fi
if test "$1" == "build_nano"; then
	arduino-cli -v compile --fqbn arduino:avr:nano:cpu=atmega328 /home/mirte/arduino_project/$2
fi
if test "$1" == "build_nano_old"; then
	arduino-cli -v compile --fqbn arduino:avr:nano:cpu=atmega328old /home/mirte/arduino_project/$2
fi
if test "$1" == "build_uno"; then
	arduino-cli -v compile --fqbn arduino:avr:uno /home/mirte/arduino_project/$2
fi

# Different upload scripts
if test "$1" == "upload" || test "$1" == "upload_stm32"; then
	arduino-cli -v upload -p /dev/ttyACM0 --fqbn STM32:stm32:GenF1:pnum=BLUEPILL_F103C8,upload_method=dfu2Method,xserial=generic,usb=CDCgen,xusb=FS,opt=osstd,rtlib=nano /home/mirte/arduino_project/$2
fi
if test "$1" == "upload_nano"; then
	arduino-cli -v upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328 /home/mirte/arduino_project/$2
fi
if test "$1" == "upload_nano_old"; then
	arduino-cli -v upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328old /home/mirte/arduino_project/$2
fi
if test "$1" == "upload_uno"; then
	arduino-cli -v upload -p /dev/ttyACM0 --fqbn arduino:avr:uno /home/mirte/arduino_project/$2
fi

if test "$1" == "upload_pico"; then
	MIRTE_SRC_DIR=/usr/local/src/mirte
	# This will always upload telemetrix4rpipico.uf2, so no need to pass a file
	sudo picotool load -f $MIRTE_SRC_DIR/mirte-install-scripts/Telemetrix4RpiPico.uf2
	retVal=$?
	if [ $retVal -ne 0 ]; then
		echo "Failed to upload to Pico"
		echo "Please check the connection and try again"
		echo "Or unplug the Pico, press the BOOTSEL button and plug it in again"
		exit 1
	fi
	sudo picotool reboot # just to make sure, sometimes it does not reboot automatically
fi

# Start ROS again
if $STOPPED_ROS; then
	sudo service mirte-ros start
	echo "STARTING ROS"
else
	echo "NOT STARTING ROS"
	echo "Start it yourself with 'sudo service mirte-ros start'"
fi
