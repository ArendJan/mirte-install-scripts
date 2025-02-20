#!/bin/bash
set -xe

MIRTE_SRC_DIR=/usr/local/src/mirte

# disable ipv6, as not all package repositories are available over ipv6
sudo tee /etc/apt/apt.conf.d/99force-ipv4 <<EOF
Acquire::ForceIPv4 "true";
EOF

# Update
sudo apt update

# Install locales
sudo apt install -y locales
sudo locale-gen "nl_NL.UTF-8"
sudo locale-gen "en_US.UTF-8"
sudo update-locale LC_ALL=en_US.UTF-8 LANGUAGE=en_US.UTF-8

# Install vcstool
cp repos.yaml $MIRTE_SRC_DIR
cp download_repos.sh $MIRTE_SRC_DIR || true
cd $MIRTE_SRC_DIR || exit 1
./download_repos.sh

# Install dependecnies to be able to run python3.8
sudo apt install -y python3.8 python3-pip python3-setuptools

# Fix for https://github.com/pypa/setuptools/issues/4478, only in python 3.8
pip3 install setuptools==70.0.0

# Set piwheels as pip repo
sudo bash -c "echo '[global]' > /etc/pip.conf"
sudo bash -c "echo 'extra-index-url=https://www.piwheels.org/simple' >> /etc/pip.conf"

# Install telemetrix
cd $MIRTE_SRC_DIR/mirte-telemetrix-aio || exit 1
pip3 install .
cd $MIRTE_SRC_DIR/mirte-tmx-pico-aio || exit 1
pip3 install .

# Install Telemtrix4Arduino project
# TODO: building STM sometimes fails (and/or hangs)
cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
mkdir -p /home/mirte/Arduino/libraries
mkdir -p /home/mirte/arduino_project/Telemetrix4Arduino
ln -s $MIRTE_SRC_DIR/mirte-telemetrix4arduino /home/mirte/Arduino/libraries/Telemetrix4Arduino
ln -s $MIRTE_SRC_DIR/mirte-telemetrix4arduino/examples/Telemetrix4Arduino/Telemetrix4Arduino.ino /home/mirte/arduino_project/Telemetrix4Arduino

# Install Mirte ROS packages and update cmake
cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
./install_ROS.sh

# Install arduino, libs & uploader (nano, stm and pico), requires cmake update from ./install_ROS.sh
cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
./install_arduino.sh

# Install Mirte Python package
cd $MIRTE_SRC_DIR/mirte-python || exit 1
pip3 install .

# Install Mirte Interface
cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
./install_web.sh

if [[ ${type:=""} != "mirte_orangepizero" ]]; then
	# Install Jupyter Notebook
	cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
	./install_jupyter_ros.sh || true # jupyter install fails on orange pi zero 1
fi

# Install overlayfs (done in sd image tools)
# Setup expand overlayfs
{
	# enable mirte-overlay service
	sudo rm /lib/systemd/system/mirte-overlay.service || true
	sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-overlay.service /lib/systemd/system/
	sudo systemctl enable mirte-overlay.service
} 2>&1 | sed -u 's/^/overlayfs::: /' &

# Install numpy
pip3 install numpy

# Install bluetooth
cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
./install_bt.sh

# Install Mirte documentation
cd $MIRTE_SRC_DIR/mirte-documentation || exit 1
sudo apt install -y python3.8-venv libenchant-dev
python3 -m venv docs-env
source docs-env/bin/activate
pip install docutils==0.16.0 sphinx-tabs==3.2.0 #TODO: use files to freeze versions
pip install wheel sphinx sphinx-prompt sphinx-rtd-theme sphinxcontrib-spelling sphinxcontrib-napoleon
mkdir -p _modules/catkin_ws/src
cd _modules || exit 1
ln -s $MIRTE_SRC_DIR/mirte-python . || true
cd mirte-python || exit 1
pip install . || true
source /opt/ros/noetic/setup.bash
source /home/mirte/mirte_ws/devel/setup.bash
cd ../../
make html || true
deactivate

# install audio support to use with mirte-pioneer pcb and orange pi zero 2
sudo apt install pulseaudio libasound2-dev libespeak1 -y
pip3 install simpleaudio pyttsx3

# Install overlayfs and make sd card read only (software)
sudo apt install -y overlayroot
# Currently only instaling, not enabled, sd-image will enable it
# remove force ipv4
sudo rm /etc/apt/apt.conf.d/99force-ipv4 || true

sudo rm /etc/resolv.conf || true # remove resolv.conf to use the one from the network.
