#!/bin/bash
set -xe
MIRTE_SRC_DIR=/usr/local/src/mirte
sudo adduser mirte dialout
echo -e "mirte_mirte\nmirte_mirte" | sudo passwd root

# Install dependencies
sudo apt install -y git curl binutils libusb-1.0-0
ls -alh $MIRTE_SRC_DIR
pip3 install -U platformio
echo "export PATH=$PATH:/home/mirte/.local/bin" >/home/mirte/.bashrc
export PATH=$PATH:/home/mirte/.local/bin
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules

mkdir -p /home/mirte/Arduino/Telemetrix4Arduino
ls $MIRTE_SRC_DIR/mirte-telemetrix4arduino -al
ln -s $MIRTE_SRC_DIR/mirte-telemetrix4arduino /home/mirte/Arduino/Telemetrix4Arduino || true
cd $MIRTE_SRC_DIR/mirte-telemetrix4arduino || exit
pio run -e robotdyn_blackpill_f303cc -e nanoatmega328new -e nanoatmega328

# pico stuff
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential libusb-1.0-0-dev libstdc++-arm-none-eabi-newlib -y
cd $MIRTE_SRC_DIR/pico-sdk/ || exit
git submodule update --init
echo "export PICO_SDK_PATH=$MIRTE_SRC_DIR/pico-sdk/" >/home/mirte/.bashrc
export PICO_SDK_PATH=$MIRTE_SRC_DIR/pico-sdk/
cd $MIRTE_SRC_DIR/mirte-telemetrix4rpipico || exit
mkdir build
cd build || exit
cmake ..
make

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

# # Already build all versions so only upload is needed
# ./run_arduino.sh build Telemetrix4Arduino
# ./run_arduino.sh build_nano Telemetrix4Arduino
# ./run_arduino.sh build_nano_old Telemetrix4Arduino
# ./run_arduino.sh build_uno Telemetrix4Arduino

# Add mirte to dialout

# By default, armbian has ssh login for root enabled with password 1234.
# The password need to be set to mirte_mirte so users can use the
# Arduino IDE remotely.
# TODO: change this to pio instead of arduino-cli

# # Enable tuploading from remote IDE
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
