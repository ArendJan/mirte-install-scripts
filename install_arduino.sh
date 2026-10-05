#!/bin/bash
set -xe
MIRTE_SRC_DIR=${MIRTE_SRC_DIR:-/usr/local/src/mirte}
export INSTALL_ARDUINO_ALL=false
. $MIRTE_SRC_DIR/settings.sh
. $MIRTE_SRC_DIR/mirte-install-scripts/tools.sh
# Install dependencies
sudo apt install -y git curl binutils libusb-1.0-0

# install platformio
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
python3 get-platformio.py
rm get-platformio.py

# Add platformio to path
export PATH=$PATH:$HOME/.local/bin
mkdir -p ~/.local/bin || true
ln -s ~/.platformio/penv/bin/platformio ~/.local/bin/platformio
ln -s ~/.platformio/penv/bin/pio ~/.local/bin/pio
ln -s ~/.platformio/penv/bin/piodebuggdb ~/.local/bin/piodebuggdb
pio --version

add_rc 'export PATH=$PATH:$HOME/.local/bin'

curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules

if [ "$INSTALL_ARDUINO_ALL" = "true" ]; then
	# Install picotool for the Raspberry Pi Pico
	sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib build-essential pkg-config libusb-1.0-0-dev cmake -y

	# Remove newlib versions that are not compatible with the pico or pico2, otherwise it takes 2GB of space
	cd /usr/lib/arm-none-eabi/newlib/thumb || true
	sudo rm -rf v8-a* || true
	sudo rm -rf v7* || true

	cd $MIRTE_SRC_DIR || exit 1
	mkdir pico/
	cd pico/ || exit 1
	git clone https://github.com/raspberrypi/pico-sdk.git --single-branch --recursive --depth=1 # somehow needed for picotool
	ls
	realpath pico-sdk
	ls
	export PICO_SDK_PATH=$MIRTE_SRC_DIR/pico/pico-sdk
	add_rc "export PICO_SDK_PATH=$MIRTE_SRC_DIR/pico/pico-sdk"

	cd $MIRTE_SRC_DIR/mirte-telemetrix4rpipico || exit 1
	git submodule update --init --recursive

else
	echo "Skipping installation of Pico tools"
	echo "Only installing tools to upload to Pico with default uf2"
	# download latest picotool for current arch linux
	arch=$(uname -m)
	curl -s https://api.github.com/repos/raspberrypi/pico-sdk-tools/releases/latest | grep -F "browser_download_url" | awk -F\" '{print $4}' | grep "picotool-.*-$arch-lin.tar.gz" | wget -i - -O /tmp/picotool-latest-$arch-lin.tar.gz
	# Check that the file was downloaded and is a valid tar.gz
	if [ ! -s /tmp/picotool-latest-$arch-lin.tar.gz ]; then
		echo "Error: Failed to download picotool tarball for architecture $arch."
		exit 1
	fi
	if ! gzip -t /tmp/picotool-latest-$arch-lin.tar.gz 2>/dev/null; then
		echo "Error: Downloaded picotool tarball is not a valid gzip file."
		exit 1
	fi
	# unzip only picotool/picotool file to /usr/local/bin/picotool
	cd /usr/local/bin || exit 1
	sudo tar -xzvf /tmp/picotool-*-$arch-lin.tar.gz picotool/picotool --strip-components=1
	sudo chmod +x ./picotool

	# uploader when using uart (mirte pioneer pcb)
	pip install -U "pip>=25" || true                                         # pico-py-serial-flash requires a newer version of pip, otherwise it'll be installed as UNKNOWN package
	pip install git+https://github.com/arendjan/pico-py-serial-flash.git@cli # uart flashing utility when using the pcb

	# download last uf2 from telemetrix pico repo
	cd $MIRTE_SRC_DIR/mirte-telemetrix4rpipico || exit 1
	# sd-image tools will try to download the correct version to .., otherwise just download the latest and hope for the best (TODO: make it more robust)
	mkdir -p build/ || true
	cd build/ || exit 1
	mv ../../Telemetrix4RpiPico.uf2 . || true
	mv ../../Telemetrix4RpiPico.elf . || true
	if [ ! -f Telemetrix4RpiPico.uf2 ]; then
		# download latest uf2 for pico from github releases
		curl -s https://api.github.com/repos/mirte-robot/telemetrix4rpipico/releases/latest | grep -F "browser_download_url" | awk -F\" '{print $4}' | grep "Telemetrix4RpiPico.*.uf2" | wget -i - -O Telemetrix4RpiPico.uf2
		curl -s https://api.github.com/repos/mirte-robot/telemetrix4rpipico/releases/latest | grep -F "browser_download_url" | awk -F\" '{print $4}' | grep "Telemetrix4RpiPico.*.elf" | wget -i - -O Telemetrix4RpiPico.elf

	fi
fi
cd $MIRTE_SRC_DIR/mirte-install-scripts/
# Already build all versions so only upload is needed *don't do for all, as it requires loads of space for the tools.
# ./run_arduino.sh build Telemetrix4Arduino
# ./run_arduino.sh upload_nano Telemetrix4Arduino # 'try to upload to the nano', to also install the upload tools.
# ./run_arduino.sh build_nano_old Telemetrix4Arduino
./run_arduino.sh build_pico
pio system prune -f
# Add mirte to dialout
sudo adduser mirte dialout
