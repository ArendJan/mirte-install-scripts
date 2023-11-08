#!/bin/bash
set -x

MIRTE_SRC_DIR=/usr/local/src/mirte
export UPDATE=false
. $MIRTE_SRC_DIR/settings.sh || (
	export INSTALL_DOCS=true
	export INSTALL_ROS=true
	export INSTALL_ARDUINO=true
	export INSTALL_WEB=true
	export INSTALL_PYTHON=true
	export INSTALL_JUPYTER=true
	export EXPIRE_PASSWD=true
	export INSTALL_NETWORK=true
	export INSTALL_PROVISIONING=true
	export PARALLEL=true
)
# Update
sudo apt update
sudo apt install -y locales python3.8 python3-pip python3-setuptools

{
	# Install locales
	sudo locale-gen "nl_NL.UTF-8"
	sudo locale-gen "en_US.UTF-8"
	sudo update-locale LC_ALL=en_US.UTF-8 LANGUAGE=en_US.UTF-8
	echo "done locale"
} 2>&1 | sed -u 's/^/locales::: /' &
if ! $PARALLEL; then
	wait
fi
# Install vcstool
pwd
ls -alh
sudo sh -c 'echo "deb http://ftp.tudelft.nl/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

# . ./download_repos.sh

# Install dependecnies to be able to run python3.8

# Set piwheels as pip repo
sudo bash -c "echo '[global]' > /etc/pip.conf"
sudo bash -c "echo 'extra-index-url=https://www.piwheels.org/simple' >> /etc/pip.conf"

if $INSTALL_ROS; then
	{
		# Install telemetrix
		cd $MIRTE_SRC_DIR/mirte-telemetrix-aio || exit
		pip3 install .
		cd $MIRTE_SRC_DIR/mirte-tmx-pico-aio || exit
		pip3 install .
		echo "done telemetrix"
	} 2>&1 | sed -u 's/^/telemetrix::: /' &
fi
if ! $PARALLEL; then
	wait
fi
if $INSTALL_ARDUINO; then
	{
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit

		. ./install_arduino.sh
		echo "done arduino"
	} 2>&1 | sed -u 's/^/arduino::: /' &
fi
if ! $PARALLEL; then
	wait
fi
if $INSTALL_PYTHON; then

	{
		# Install Mirte Python package
		cd $MIRTE_SRC_DIR/mirte-python || exit
		pip3 install .
		echo "done mirte-python"
	} 2>&1 | sed -u 's/^/mirte-python::: /' &
fi
if ! $PARALLEL; then
	wait
fi
if $INSTALL_WEB; then

	{
		# Install Mirte Interface
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit
		. ./install_web.sh
		echo "done web"
	} 2>&1 | sed -u 's/^/web::: /' &
fi
if ! $PARALLEL; then
	wait
fi
if $INSTALL_JUPYTER; then

	{
		# Install Jupyter Notebook
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit
		. ./install_jupyter_ros.sh
		echo "done jupyter_ros"
	} 2>&1 | sed -u 's/^/jupyter_ros::: /' &
fi
if ! $PARALLEL; then
	wait
fi

if $INSTALL_DOCS; then

	# Install Mirte documentation
	{
		. ./install_docs.sh
		echo "done docs"
	} 2>&1 | sed -u 's/^/docs::: /' &
fi
if ! $PARALLEL; then
	wait
fi
if $INSTALL_PROVISIONING; then

	# Install Mirte provisioning system
	{
		sudo pip install watchdog pyyaml nmcli
		sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/provisioning/mirte-provisioning.service /lib/systemd/system/
		sudo systemctl enable mirte-provisioning.service
		echo "done provisioning"
	} 2>&1 | sed -u 's/^/provisioning::: /' &
fi
if ! $PARALLEL; then
	wait
fi

if $INSTALL_ROS; then
	wait # rosdep does wait for other apt scripts to finish, then it just fails installing. If we wait for the others to finish, there won't be parralel apt scripts running.
	{
		# Install Mirte ROS packages
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit
		. ./install_ROS.sh
		echo "done ROS"
	} 2>&1 | sed -u 's/^/ROS::: /' &
	wait
fi
# Install overlayfs and make sd card read only (software)
# sudo apt install -y overlayroot
# Currently only instaling, not enabled
#sudo bash -c "echo 'overlayroot=\"tmpfs\"' >> /etc/overlayroot.conf"

# Install numpy
pip3 install numpy

sudo apt install -y bluez joystick
# if [ "$(uname -a | grep sunxi)" != "" ]; then
# 	# currently only supporting cheap USB dongles on OrangePi
# 	. ./install_fake_bt.sh
# fi
./install_pico.sh

# Install overlayfs and make sd card read only (software)
sudo apt install -y overlayroot
# Currently only instaling, not enabled
#sudo bash -c "echo 'overlayroot=\"tmpfs\"' >> /etc/overlayroot.conf"

cd $MIRTE_SRC_DIR/mirte-install-scripts/pam/password_manager/ || true
python3 -m pip install -r requirements.txt
sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-passwords.service /lib/systemd/system/
sudo systemctl enable mirte-passwords

echo "Cleaning cache"
sudo du -sh /var/cache/apt/archives
sudo apt clean

echo "Waiting"
time wait # wait on all the backgrounded stuff
echo "Done installing"
# cd /home/mirte/
date >install_date.txt

