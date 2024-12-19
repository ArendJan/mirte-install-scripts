#!/bin/bash
set -xe
MIRTE_SRC_DIR=/usr/local/src/mirte

. $MIRTE_SRC_DIR/mirte-install-scripts/tools.sh
# Install dependencies
sudo apt install -y git curl binutils libusb-1.0-0

# install platformio
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
python3 get-platformio.py

# Add platformio to path
export PATH=$PATH:$HOME/.local/bin
mkdir -p ~/.local/bin || true
ln -s ~/.platformio/penv/bin/platformio ~/.local/bin/platformio
ln -s ~/.platformio/penv/bin/pio ~/.local/bin/pio
ln -s ~/.platformio/penv/bin/piodebuggdb ~/.local/bin/piodebuggdb
pio --version

add_rc 'export PATH=$PATH:$HOME/.local/bin'
# Install picotool for the Raspberry Pi Pico
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib build-essential pkg-config libusb-1.0-0-dev cmake -y
cd $MIRTE_SRC_DIR || exit 1
mkdir pico/
cd pico/ || exit 1
git clone https://github.com/raspberrypi/pico-sdk.git --recursive # somehow needed for picotool
ls
realpath pico-sdk
ls
export PICO_SDK_PATH=$MIRTE_SRC_DIR/pico/pico-sdk
add_rc "export PICO_SDK_PATH=$MIRTE_SRC_DIR/pico/pico-sdk"
git clone https://github.com/raspberrypi/picotool.git
cd picotool || exit 1
sudo cp udev/99-picotool.rules /etc/udev/rules.d/

mkdir build
cd build || exit 1
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install
cd $MIRTE_SRC_DIR/mirte-install-scripts/
# Already build all versions so only upload is needed
./run_arduino.sh build Telemetrix4Arduino
./run_arduino.sh build_nano Telemetrix4Arduino
./run_arduino.sh build_nano_old Telemetrix4Arduino
./run_arduino.sh build_pico
# Add mirte to dialout
sudo adduser mirte dialout
