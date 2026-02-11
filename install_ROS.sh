#!/bin/bash
set -xe
#TODO: get this as a parameter
MIRTE_SRC_DIR=/usr/local/src/mirte

# There is a bug with Cmake in qemu for armhf:
# https://gitlab.kitware.com/cmake/cmake/-/issues/20568
# So we need to install a newer version of Cmake
# https://apt.kitware.com/
wget https://apt.kitware.com/kitware-archive.sh
chmod +x kitware-archive.sh
sudo ./kitware-archive.sh
rm kitware-archive.sh
sudo apt update
sudo apt install cmake=3.31.5 -y
# Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-ros-base python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools python3-osrf-pycommon
grep -qxF "source /opt/ros/noetic/setup.bash" /home/mirte/.bashrc || echo "source /opt/ros/noetic/setup.bash" >>/home/mirte/.bashrc
source /opt/ros/noetic/setup.bash
sudo rosdep init
rosdep update --rosdistro=noetic

# Install computer vision libraries
#TODO: make dependecies of ROS package
sudo apt install -y python3-pip python3-wheel python3-setuptools python3-opencv libzbar0
sudo pip3 install pyzbar

# Move custom settings to writabel filesystem
#cp $MIRTE_SRC_DIR/mirte-ros-packages/mirte_telemetrix/config/mirte_user_settings.yaml /home/mirte/.user_settings.yaml
#rm $MIRTE_SRC_DIR/mirte-ros-packages/mirte_telemetrix/config/mirte_user_settings.yaml
#ln -s /home/mirte/.user_settings.yaml $MIRTE_SRC_DIR/mirte-ros-packages/config/mirte_user_settings.yaml

# Install Mirte ROS package
mkdir -p /home/mirte/mirte_ws/src
cd /home/mirte/mirte_ws/src || exit 1
ln -s $MIRTE_SRC_DIR/mirte-ros-packages .
cd ..
rosdep install -y --from-paths src/ --ignore-src --rosdistro noetic
catkin build
grep -qxF "source /home/mirte/mirte_ws/devel/setup.bash" /home/mirte/.bashrc || echo "source /home/mirte/mirte_ws/devel/setup.bash" >>/home/mirte/.bashrc
source /home/mirte/mirte_ws/devel/setup.bash

# install missing python dependencies rosbridge
#sudo apt install -y libffi-dev libjpeg-dev zlib1g-dev
#sudo pip3 install twisted pyOpenSSL autobahn tornado pymongo

# Add systemd service to start ROS nodes
sudo rm /lib/systemd/system/mirte-ros.service || true
sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-ros.service /lib/systemd/system/

sudo systemctl daemon-reload
sudo systemctl stop mirte-ros || /bin/true
sudo systemctl start mirte-ros
sudo systemctl enable mirte-ros

sudo usermod -a -G video mirte

# Install OLED dependencies (adafruit dependecies often break, so explicityle set to versions)
sudo apt install -y python3-bitstring libfreetype6-dev libjpeg-dev zlib1g-dev fonts-dejavu
sudo pip3 install adafruit-circuitpython-busdevice==5.1.1 adafruit-circuitpython-framebuf==1.4.9 adafruit-circuitpython-typing==1.7.0 Adafruit-PlatformDetect==3.22.1
sudo pip3 install pillow adafruit-circuitpython-ssd1306==2.12.1

# Install aio dependencies
sudo pip3 install janus async-generator nest-asyncio
git clone https://github.com/locusrobotics/aiorospy.git
cd aiorospy/aiorospy || exit 1
sudo pip3 install .
cd ../..
sudo rm -rf aiorospy
