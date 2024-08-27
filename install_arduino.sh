#!/bin/bash
set -xe
MIRTE_SRC_DIR=/usr/local/src/mirte

# Install dependencies
sudo apt install -y git curl binutils libusb-1.0-0

# Disabled arduino build as we use pico and it often hangs the build
# # Install arduino-cli
# # We need to install version 0.13.0. From version 0.14.0 on a check is done on the hash of the packages,
# # while the community version of the STM (see below) needs insecure packages.
# curl https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sudo BINDIR=/usr/local/bin sh -s 0.13.0

# # Install arduino avr support (for nano)
# arduino-cli -v core update-index --additional-urls https://raw.githubusercontent.com/koendv/stm32duino-raspberrypi/master/BoardManagerFiles/package_stm_index.json
# arduino-cli -v core install arduino:avr

# # Install STM32 support. Currently not supported by stm32duino (see https://github.com/stm32duino/Arduino_Core_STM32/issues/708), but there is already
# # a community version (https://github.com/koendv/stm32duino-raspberrypi). TODO: go back to stm32duino as soon as it is merged into stm32duino.
# arduino-cli -v core install STM32:stm32 --additional-urls https://github.com/koendv/stm32duino-raspberrypi/blob/v1.3.2-4/BoardManagerFiles/package_stm_index.json
# #arduino-cli -v core install STM32:stm32 --additional-urls https://github.com/zoef-robot/stm32duino-raspberrypi/master/BoardManagerFiles/package_stm_index.json

# # Fix for community STM32 (TODO: make version independant)
# sed -i 's/dfu-util\.sh/dfu-util\/dfu-util/g' /home/mirte/.arduino15/packages/STM32/tools/STM32Tools/1.4.0/tools/linux/maple_upload
# ln -s /home/mirte/.arduino15/packages/STM32/tools/STM32Tools/1.4.0/tools/linux/maple_upload /home/mirte/.arduino15/packages/STM32/tools/STM32Tools/1.4.0/tools/linux/maple_upload.sh
# sudo cp /home/mirte/.arduino15/packages/STM32/tools/STM32Tools/1.4.0/tools/linux/45-maple.rules /etc/udev/rules.d/45-maple.rules
# # Retartsing should only be done when not in qemu
# #sudo service udev restart

# # Install libraries needed by FirmataExpress
# arduino-cli lib install "NewPing"
# arduino-cli lib install "Stepper"
# arduino-cli lib install "Servo"
# arduino-cli lib install "DHTNEW"

# # Install our own arduino libraries
# ln -s $MIRTE_SRC_DIR/mirte-arduino-libraries/OpticalEncoder /home/mirte/Arduino/libraries

# # Install Blink example code
# mkdir /home/mirte/arduino_project/Blink
# ln -s $MIRTE_SRC_DIR/mirte-install-scripts/Blink.ino /home/mirte/arduino_project/Blink

# Already build all versions so only upload is needed
# TODO: somehow this takes ages, removed for now:
# ./run_arduino.sh build Telemetrix4Arduino
# ./run_arduino.sh build_nano Telemetrix4Arduino
# ./run_arduino.sh build_nano_old Telemetrix4Arduino
# ./run_arduino.sh build_uno Telemetrix4Arduino

# Add mirte to dialout
sudo adduser mirte dialout

# By default, armbian has ssh login for root enabled with password 1234.
# The password need to be set to mirte_mirte so users can use the
# Arduino IDE remotely.
# TODO: when the Arduino IDE also supports ssh for non-root-users
# this has to be changed
echo -e "mirte_mirte\nmirte_mirte" | sudo passwd root

# Enable tuploading from remote IDE
# sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/run-avrdude /usr/bin
# sudo bash -c 'echo "mirte ALL = (root) NOPASSWD: /usr/local/bin/arduino-cli" >> /etc/sudoers'

# Install picotool for the Raspberry Pi Pico
sudo apt install build-essential pkg-config libusb-1.0-0-dev cmake -y
cd /tmp/ || exit 1
git clone https://github.com/raspberrypi/pico-sdk.git # somehow needed for picotool
export PICO_SDK_PATH=/tmp/pico-sdk
git clone https://github.com/raspberrypi/picotool.git
cd picotool || exit 1
sudo cp udev/99-picotool.rules /etc/udev/rules.d/

mkdir build
cd build || exit 1
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install

cd /tmp || exit 1
rm -rf pico-sdk
rm -rf picotool

#  Download latest uf2 release, resulting in Telemetrix4RpiPico.uf2
# TODO:  Downlaods from arendjan/telemetrix4rpipico, as it isn't released yet on the official repo
cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
# curl -s https://api.github.com/repos/arendjan/telemetrix4rpipico/releases/latest |
# 	grep ".*/Telemetrix4RpiPico.uf2" |
# 	cut -d : -f 2,3 |
# 	tr -d \" |
# 	wget -qi -

wget https://mirte.arend-jan.com/files/telemetrix/modules2/Telemetrix4RpiPico.uf2
