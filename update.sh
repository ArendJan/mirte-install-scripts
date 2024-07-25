#!/bin/bash

# TODO: Untested and unused for now
MIRTE_SRC_DIR=/usr/local/src/mirte

export UPDATE=true

# TODO: update git packages

{
	# Install telemetrix
	cd $MIRTE_SRC_DIR/mirte-telemetrix-aio || exit
	pip3 install .
	echo "done telemetrix"
} 2>&1 | sed -u 's/^/telemetrix::: /' &

{
	cd $MIRTE_SRC_DIR/mirte-install-scripts || exit

	. ./install_arduino.sh
	echo "done arduino"
} 2>&1 | sed -u 's/^/arduino::: /' &

if true; then
	{
		# Install Mirte Python package
		cd $MIRTE_SRC_DIR/mirte-python || exit
		pip3 install .
		echo "done mirte-python"
	} 2>&1 | sed -u 's/^/mirte-python::: /' &

	{
		# Install Mirte Interface
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit
		. ./install_web.sh
		echo "done web"
	} 2>&1 | sed -u 's/^/web::: /' &

	{
		# Install Jupyter Notebook
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit
		. ./install_jupyter_ros.sh
		echo "done jupyter_ros"
	} 2>&1 | sed -u 's/^/jupyter_ros::: /' &

	{
		# Install Mirte ROS packages
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit
		. ./install_ROS.sh
		echo "done ROS"
	} 2>&1 | sed -u 's/^/ROS::: /' &

	# Install Mirte documentation
	{
		. ./install_docs.sh
		echo "done docs"
	} 2>&1 | sed -u 's/^/docs::: /' &
# Install overlayfs and make sd card read only (software)
# sudo apt install -y overlayroot
# Currently only instaling, not enabled
#sudo bash -c "echo 'overlayroot=\"tmpfs\"' >> /etc/overlayroot.conf"
fi
echo "Waiting"
time wait # wait on all the backgrounded stuff
echo "Done installing"
