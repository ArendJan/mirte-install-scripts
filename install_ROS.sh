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
until sudo apt update; do
	echo "retrying apt update in 1s"
	sleep 1
done
sudo apt install cmake -y
# Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

until sudo apt update; do
	echo "retrying apt update in 1s"
	sleep 1
done
sudo apt install -y ros-noetic-ros-base python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools python3-osrf-pycommon
grep -qxF "source /opt/ros/noetic/setup.bash" /home/mirte/.bashrc || echo "source /opt/ros/noetic/setup.bash" >>/home/mirte/.bashrc
grep -qxF "source /opt/ros/noetic/setup.zsh" /home/mirte/.zshrc || echo "source /opt/ros/noetic/setup.zsh" >>/home/mirte/.zshrc
source /opt/ros/noetic/setup.bash
sudo rosdep init || true # init will fail if it is already initialized
rosdep update

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

if [[ $MIRTE_TYPE == "mirte-master" ]]; then
	# install lidar and depth camera
	cd /home/mirte/mirte_ws/src || exit 1
	git clone https://github.com/Slamtec/rplidar_ros.git
	git clone https://github.com/arendjan/ros_astra_camera.git -b fix-image-transport # compressed images
	git clone https://github.com/arendjan/ridgeback.git
	cd ../../
	mkdir temp
	cd temp || exit 1
	sudo apt install -y libudev-dev libusb-1.0-0-dev libjpeg-dev
	sudo ldconfig
	git clone https://github.com/libuvc/libuvc.git
	cd libuvc
	mkdir build && cd build
	cmake .. && make -j4
	sudo make install
	sudo ldconfig
	cd ../../../
	sudo rm -rf temp
	cd /home/mirte/mirte_ws/ || exit 1
	rosdep install -y --from-paths src/ --ignore-src --rosdistro noetic
	catkin build
	source ./devel/setup.bash
	roscd astra_camera
	./scripts/create_udev_rules
	sudo udevadm control --reload && sudo udevadm trigger
	roscd rplidar_ros
	./scripts/create_udev_rules.sh
fi

# -r is for continue on error, mirte-ros-packages references astra_camera which is not installed on default images.
rosdep install -y --from-paths src/ --ignore-src --rosdistro noetic -r
catkin build
grep -qxF "source /home/mirte/mirte_ws/devel/setup.bash" /home/mirte/.bashrc || echo "source /home/mirte/mirte_ws/devel/setup.bash" >>/home/mirte/.bashrc
grep -qxF "source /home/mirte/mirte_ws/devel/setup.zsh" /home/mirte/.zshrc || echo "source /home/mirte/mirte_ws/devel/setup.zsh" >>/home/mirte/.zshrc

source /home/mirte/mirte_ws/devel/setup.bash

# install missing python dependencies rosbridge
#sudo apt install -y libffi-dev libjpeg-dev zlib1g-dev
#sudo pip3 install twisted pyOpenSSL autobahn tornado pymongo

# Add systemd service to start ROS nodes
ROS_SERVICE_NAME=mirte-ros
if [[ $MIRTE_TYPE == "mirte-master" ]]; then # master version should start a different launch file
	# ROS_SERVICE_NAME=mirte-master-ros # somehow this doesn't work nicely, so we just duplicate the file for the master.
	rm $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-ros.service
	cp $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-master-ros.service $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-ros.service
fi
sudo rm /lib/systemd/system/mirte-ros.service || true
sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-ros.service /lib/systemd/system/mirte-ros.service

sudo systemctl daemon-reload
sudo systemctl stop mirte-ros.service || /bin/true
sudo systemctl start mirte-ros.service
sudo systemctl enable mirte-ros.service

sudo usermod -a -G video mirte

# Install OLED dependencies (adafruit dependecies often break, so explicityle set to versions)
sudo apt install -y python3-bitstring libfreetype6-dev libjpeg-dev zlib1g-dev fonts-dejavu
sudo pip3 install adafruit-circuitpython-busdevice==5.1.1 adafruit-circuitpython-framebuf==1.4.9 adafruit-circuitpython-typing==1.7.0 Adafruit-PlatformDetect==3.22.1
sudo pip3 install pillow adafruit-circuitpython-ssd1306==2.12.1

# Install aio dependencies
sudo pip3 install janus async-generator nest-asyncio catkin_pkg
git clone https://github.com/locusrobotics/aiorospy.git
cd aiorospy/aiorospy || exit 1
sudo pip3 install .
cd ../..
sudo rm -rf aiorospy
