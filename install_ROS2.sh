#!/bin/bash
set -xe
# IMPORTANT:
# Do not upgrade apt-get since it will break the image. libc-bin will for some
# reason break and not be able to install new stuff on the image.
# TODO: check above info, no issues yet (2024-12-11)

#TODO: get this as a parameter
MIRTE_SRC_DIR=${MIRTE_SRC_DIR:-/usr/local/src/mirte}
. tools.sh
# shellcheck source=/dev/null
source /etc/os-release

# Install ROS 2 (Humble)
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y

# check if ros2-apt-source is already installed, if not, install it
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then

	ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
	if [ -z "$ROS_APT_SOURCE_VERSION" ]; then
		ROS_APT_SOURCE_VERSION="1.2.0" # when curl fails, use a fallback version, latest as of april 2026
	fi
	curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
	sudo apt install /tmp/ros2-apt-source.deb
fi
sudo apt update

# select ros2 type based on UBUNTU_CODENAME
if [[ $UBUNTU_CODENAME == "jammy" ]]; then
	ROS_NAME=humble
elif [[ $UBUNTU_CODENAME == "noble" ]]; then
	ROS_NAME=jazzy
else
	echo "Unknown UBUNTU_CODENAME: $UBUNTU_CODENAME"
	exit 1
fi

sudo apt install -y ros-$ROS_NAME-ros-base ros-$ROS_NAME-zenoh-bridge-dds ros-$ROS_NAME-rmw-zenoh-cpp
sudo apt install -y ros-$ROS_NAME-xacro
sudo apt install -y ros-dev-tools ccache

# shellcheck source=/dev/null
source /opt/ros/$ROS_NAME/setup.bash
sudo rosdep init
rosdep update

# Install computer vision libraries
#TODO: make dependecies of ROS package
sudo apt install -y python3-pip python3-wheel python3-setuptools python3-opencv libzbar0
sudo pip3 install pyzbar mergedeep

# TODO: move configs to mirte bringup
#cp $MIRTE_SRC_DIR/mirte-ros-packages/mirte_telemetrix/config/mirte_user_settings.yaml /home/mirte/.user_settings.yaml
#rm $MIRTE_SRC_DIR/mirte-ros-packages/mirte_telemetrix/config/mirte_user_settings.yaml
#ln -s /home/mirte/.user_settings.yaml $MIRTE_SRC_DIR/mirte-ros-packages/config/mirte_user_settings.yaml

# TODO: install in a separate workspace or install the debs.
# Install Mirte ROS package
mkdir -p /home/mirte/mirte_ws/src
cd /home/mirte/mirte_ws/src
ln -s $MIRTE_SRC_DIR/mirte-ros-packages .

sudo apt install python3-colcon-mixin -y
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

# Add colcon_defaults.yaml to the workspace, symlink and ccache
cat <<EOF >>/home/mirte/mirte_ws/colcon_defaults.yaml
# Default colcon build options for the Mirte workspace
build:
  symlink-install: true
  mixin: 
    - "ccache"
EOF

# if mirte-ros-packages is from main or develop, use the precompiled version, otherwise compile on-device
cd $MIRTE_SRC_DIR/mirte-ros-packages
git submodule update --init --recursive
branch=$(git rev-parse --abbrev-ref HEAD)
arch=$(dpkg --print-architecture)
ubuntu_version=$(lsb_release -cs)
github_url=$(git config --get remote.origin.url | sed 's/\.git$//')
fallback=true
cd /home/mirte/mirte_ws/ || exit 1
vcs import src <$MIRTE_SRC_DIR/mirte-ros-packages/sources.repos || true
if [[ $branch == "develop" || $branch == "main" ]]; then
	fallback=false

	# Install mirte ros packages with apt from github, since they take ages to compile and it's easier to update them.
	# colcon ignore those packages
	if [[ $branch == "develop" ]]; then
		arch="${arch}_develop"
	fi
	echo "Using precompiled version of packages"
	echo "deb [trusted=yes] $github_url/raw/ros_mirte_${ROS_NAME}_${ubuntu_version}_${arch}/ ./" | sudo tee /etc/apt/sources.list.d/mirte-ros-packages.list
	echo "yaml $github_url/raw/ros_mirte_${ROS_NAME}_${ubuntu_version}_${arch}/local.yaml ${ROS_NAME}" | sudo tee /etc/ros/rosdep/sources.list.d/mirte-ros-packages.list
	sudo apt update

	cd /home/mirte/mirte_ws/src/ || exit 1
	ignore=(mirte_telemetrix_cpp mirte_msgs mirte_teleop astra_camera astra_camera_msgs libuvc mirte_base_control mirte_master_arm_control mirte_control usb_cam)
	packages=''
	for i in "${ignore[@]}"; do
		path=$(colcon list --packages-select $i -p)
		# if path is empty, skip
		if [[ -z $path ]]; then
			echo "Package $i not found, skipping."
			continue
		fi
		path=$(realpath "$path")
		echo "Checking package $i at $path"
		if [[ ! -d $path ]]; then
			echo "Package $i not found, skipping."
			continue
		fi
		touch $path/COLCON_IGNORE
		i_dash=$(echo $i | tr '_' '-')
		pkg_name="ros-$ROS_NAME-$i_dash"
		# check if dbgsym package exists
		sudo apt-cache madison "$pkg_name-dbgsym"

		if sudo apt-cache madison "$pkg_name-dbgsym" | grep -vqz "Unable to locate package"; then
			echo "Adding debug package $pkg_name-dbgsym"
			packages="$packages $pkg_name-dbgsym"
		fi
		packages="$packages $pkg_name"
	done
	sudo apt install -y -m $packages || fallback=false # TODO: disabled fallback for now as mirte-arm doesn't compile.
fi

if $fallback; then
	echo "Compiling mirte-ros-packages on-device"
	cd /home/mirte/mirte_ws/src/mirte-ros-packages || exit 1
	find . -name "COLCON_IGNORE" -type f -delete
fi

sudo apt install libboost-all-dev -y
cd /home/mirte/mirte_ws/src || exit 1
# git clone https://github.com/AlexKaravaev/ros2_laser_scan_matcher
# git clone https://github.com/AlexKaravaev/csm
# git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
# git clone https://github.com/RobotWebTools/web_video_server.git -b ros2
cd .. || exit 1
rosdep install -y --from-paths src/ --ignore-src --rosdistro $ROS_NAME
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
add_mirte_settings "export MIRTE_ZENOH=false"          # disable zenoh by default, but can be enabled by setting to true
add_mirte_settings "export MIRTE_USE_MULTIROBOT=false" # TODO: use this in the launch file?
add_mirte_settings "export ROS_LOG_DIR=/tmp/ros"
add_rc "source /home/mirte/.mirte_settings.sh"
add_rc "# Enable Zenoh and multirobot in .mirte_settings.sh"

add_rc "source /home/mirte/mirte_ws/install/setup.bash" "# sourced later on"

# By default the ROS communication should only be localhost.
add_mirte_settings "export ROS_LOCALHOST_ONLY=1"

# shellcheck source=/dev/null
source /home/mirte/mirte_ws/install/setup.bash

# Add systemd service to start ROS nodes
if [[ $MIRTE_TYPE == "mirte-master" ]]; then # master version should start a different launch file
	add_mirte_settings "export ROS_LOCALHOST_ONLY=0"
	# write mirte type to file, so that the mirte_ros service can read it and start the correct launch file
	cat <<EOF >$MIRTE_SRC_DIR/mirte-install-scripts/services/mirte_ros_type.sh
export MIRTE_TYPE=mirte-master
EOF
	update_machine_config "type" "mirte-master"
fi

# The service file starts mirte_ros, which reads the mirte_ros_type.sh file to determine which launch file to use.
add_service mirte-ros.service

sudo usermod -a -G video mirte
sudo adduser mirte dialout

# Some nice extra packages: clean can clean workspaces and packages. No need to do it by hand. lint can check for errors in the cmake/package code.
sudo pip3 install colcon-clean colcon-lint

if [[ $MIRTE_TYPE == "mirte-master" ]]; then
	# TODO: need to check and edit the next part:
	sudo apt install ros-$ROS_NAME-slam-toolbox -y

	# install lidar and depth camera
	cd /home/mirte/mirte_ws/src || exit 1
	git clone https://github.com/ArendJan/rplidar_ros.git --single-branch -b auto_standby_launch_param_c1 # FIXME-FUTURE: When merged, switch back to official repo

	# git clone https://github.com/ArendJan/ros2_astra_camera.git -b fix-ros-jammy      # compressed images image transport fixes, fork of orbbec/... with also lazy nodes
	# git clone https://github.com/clearpathrobotics/clearpath_mecanum_drive_controller # FIXME: Can be installed from apt? why build?
	# cd ../../
	# mkdir temp
	# cd temp || exit 1
	# sudo apt install -y libudev-dev libusb-1.0-0-dev nlohmann-json3-dev
	# # Install lubuvc-dev manually for newer version
	# git clone https://github.com/libuvc/libuvc.git
	# cd libuvc
	# mkdir build && cd build
	# cmake .. && make -j4
	# sudo make install
	# sudo ldconfig
	# cd ../../../
	# sudo rm -rf temp
	cd /home/mirte/mirte_ws/ || exit 1
	rosdep install -y --from-paths src/ --ignore-src --rosdistro $ROS_NAME
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
	# shellcheck source=/dev/null
	source ./install/setup.bash
	cd src/ros2_astra_camera/astra_camera
	chmod +x ./scripts/install.sh || true
	sudo ./scripts/install.sh || true
	sudo udevadm control --reload && sudo udevadm trigger
	cd ../../rplidar_ros
	chmod +x ./scripts/create_udev_rules.sh || true
	./scripts/create_udev_rules.sh || true
	cd /home/mirte/mirte_ws/ || exit 1
	cd src/orbbecsdk_ros2/ || true
	sudo ./orbbec_camera/scripts/install_udev_rules.sh || true

fi
# custom version of teleop-twist-keyboard, since the original one jitters at start.
# self-built version starts with 2-2026.....
# https://github.com/ArendJan/teleop_twist_keyboard/tree/add-keyboard-timeout-and-rate-params
sudo apt install -y --allow-downgrades "ros-$ROS_NAME-teleop-twist-keyboard=2-*"
# prevent it from being upgraded, it isn't updated anyways.
sudo apt-mark hold "ros-$ROS_NAME-teleop-twist-keyboard"

# Add colcon top level workspace, this makes it possible to run colcon build from any folder, it will find the workspace and build it. Otherwise it will create a new workspace in the subdirectory.
pip install git+https://github.com/arendjan/colcon-top-level-workspace.git

# zsh does not work nicely with ros2 autocomplete, so we need to add a function to fix it.
# ROS 2 Foxy should have this fixed, but we are using ROS 2 Humble.
# TODO: check for ROS2 jazzy
cat <<EOF >>/home/mirte/.zshrc
sr () { # macro to source the workspace and enable autocompletion. sr stands for source ros, no other command should use this abbreviation.
    . /opt/ros/humble/setup.zsh
    . ~/mirte_ws/install/setup.zsh
    eval "\$(register-python-argcomplete3 ros2)"
    eval "\$(register-python-argcomplete3 colcon)"
}
cb () {
    pkg=\$1
    # if package not empty
    if [ -n "\$pkg" ]; then
        colcon build --symlink-install --packages-up-to \$pkg
    else
        colcon build --symlink-install 
    fi
}
cbr () {
    pkg=\$1
    # if package not empty
    if [ -n "\$pkg" ]; then
        colcon build --symlink-install --packages-up-to \$pkg --cmake-args -DCMAKE_BUILD_TYPE=Release
    else
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    fi
}
sr
EOF
