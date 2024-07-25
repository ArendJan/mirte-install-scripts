#!/bin/bash
set -xe

MIRTE_SRC_DIR=/usr/local/src/mirte
. $MIRTE_SRC_DIR/settings.sh || (
	export INSTALL_DOCS=true
	export INSTALL_ROS=true
	export INSTALL_ARDUINO=true
	export INSTALL_WEB=true
	export BUILD_WEB=true
	export INSTALL_PYTHON=true
	export INSTALL_JUPYTER=true
	export EXPIRE_PASSWD=true
	export INSTALL_NETWORK=true
	export INSTALL_PROVISIONING=true
	export PARALLEL=true
	export MIRTE_TYPE=default # default, mirte-master

)

wait_all() {
	while [ "$(jobs -p | wc -l)" -gt 0 ]; do # wait for all backgrounded jobs to finish
		state=0
		wait -n || state=$? # wait for next job to finish. "||" is required to not trigger set -e
		if [[ $state -ne 0 ]]; then
			pkill -P $$ || true # kill all child processes
			exit 1              # exit with error
		fi
	done
}

# disable ipv6, as not all package repositories are available over ipv6
sudo tee /etc/apt/apt.conf.d/99force-ipv4 <<EOF
Acquire::ForceIPv4 "true";
EOF

# Update
sudo apt update || true
sudo apt install -y locales python3.8 python3-pip python3-setuptools

{
	# Install locales
	sudo locale-gen "nl_NL.UTF-8"
	sudo locale-gen "en_US.UTF-8"
	sudo update-locale LC_ALL=en_US.UTF-8 LANGUAGE=en_US.UTF-8
	echo "done locale"
} 2>&1 | sed -u 's/^/locales::: /' &
if ! $PARALLEL; then
	wait_all
fi
# Install vcstool
cp repos.yaml $MIRTE_SRC_DIR
cp download_repos.sh $MIRTE_SRC_DIR || true
cd $MIRTE_SRC_DIR || exit 1
. ./download_repos.sh

# Install dependencies to be able to run python3.8
pip3 install setuptools --upgrade
# Set piwheels as pip repo
sudo bash -c "echo '[global]' > /etc/pip.conf"
sudo bash -c "echo 'extra-index-url=https://www.piwheels.org/simple' >> /etc/pip.conf"

if $INSTALL_ROS; then
	{
		# Install telemetrix
		cd $MIRTE_SRC_DIR/mirte-telemetrix-aio || exit 1
		pip3 install .
		cd $MIRTE_SRC_DIR/mirte-tmx-pico-aio || exit 1
		pip3 install .
		echo "done telemetrix"
	} 2>&1 | sed -u 's/^/telemetrix::: /' &
fi
if ! $PARALLEL; then
	wait_all
fi
if $INSTALL_ARDUINO; then
	{
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1

		. ./install_arduino.sh
		echo "done arduino"
	} 2>&1 | sed -u 's/^/arduino::: /' &
fi
if ! $PARALLEL; then
	wait_all
fi
if $INSTALL_PYTHON; then

	{
		# Install Mirte Python package
		cd $MIRTE_SRC_DIR/mirte-python || exit 1
		pip3 install .
		echo "done mirte-python"
	} 2>&1 | sed -u 's/^/mirte-python::: /' &
fi
if ! $PARALLEL; then
	wait_all
fi
if $INSTALL_WEB; then

	{
		# Install Mirte Interface
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
		. ./install_web.sh
		echo "done web"
	} 2>&1 | sed -u 's/^/web::: /' &
fi
if ! $PARALLEL; then
	wait_all
fi
if $INSTALL_JUPYTER; then

	{
		if [[ ${type:=""} != "mirte_orangepizero" ]]; then
			# Install Jupyter Notebook
			cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
			. ./install_jupyter_ros.sh || true # jupyter install fails on orange pi zero 1
		fi
		echo "done jupyter_ros"
	} 2>&1 | sed -u 's/^/jupyter_ros::: /' &
fi
if ! $PARALLEL; then
	wait_all
fi

if $INSTALL_VSCODE; then
	{
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
		. ./install_vscode.sh || exit 1
		echo "done VSCode"
	} 2>&1 | sed -u 's/^/vscode::: /' &
fi
if ! $PARALLEL; then
	wait_all
fi

if $INSTALL_DOCS; then

	# Install Mirte documentation
	{
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
		. ./install_docs.sh || true # docs building is a bit flaky
		echo "done docs"
	} 2>&1 | sed -u 's/^/docs::: /' &
fi
if ! $PARALLEL; then
	wait_all
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
	wait_all
fi

if $INSTALL_ROS; then
	wait_all # rosdep doesnt wait for other apt scripts to finish, then it just fails installing. If we wait for the others to finish, there won't be parralel apt scripts running.
	{
		# Install Mirte ROS packages
		cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
		. ./install_ROS.sh
		echo "done ROS"
	} 2>&1 | sed -u 's/^/ROS::: /' &
	wait_all
fi
# Install overlayfs and make sd card read only (software)
# sudo apt install -y overlayroot
# Currently only instaling, not enabled
#sudo bash -c "echo 'overlayroot=\"tmpfs\"' >> /etc/overlayroot.conf"

# Install numpy
pip3 install numpy
cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
./install_bt.sh

# if building for mirte-master:
if [[ $MIRTE_TYPE == "mirte-master" ]]; then

	# set default password for root to ...
	sudo sed -i '/^root:/d' /etc/shadow
	echo 'root:$6$iPpuScKGQTiuJk9r$cBXX/s.8UBp0bvrshHRhw/tHcmU3.beHBfCyJgP8Qhjx2CEO5.dyyvKips6loYQocSTgS/qEYxPrOQd/.qVi70:19793:0:99999:7:::' | sudo tee -a /etc/shadow
	# Install Mirte Master
	cd $MIRTE_SRC_DIR/mirte-install-scripts || exit 1
	./install_mirte_master.sh
fi

# install audio support to use with mirte-pioneer pcb and orange pi zero 2
sudo apt install pulseaudio libasound2-dev libespeak1 -y
pip3 install simpleaudio pyttsx3 || true # allow failing install

# Install overlayfs and make sd card read only (software)
sudo apt install -y overlayroot
# Currently only instaling, not enabled
#sudo bash -c "echo 'overlayroot=\"tmpfs\"' >> /etc/overlayroot.conf"

# remove force ipv4
sudo rm /etc/apt/apt.conf.d/99force-ipv4 || true

echo "Cleaning cache"
sudo du -sh /var/cache/apt/archives
sudo apt clean

echo "Waiting"
time wait_all # wait on all the backgrounded stuff
echo "Done installing"
# cd /home/mirte/
date >install_date.txt
