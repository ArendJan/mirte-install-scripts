#!/bin/bash
set -xe
# IMPORTANT:
# Do not upgrade apt-get since it will break the image. libc-bin will for some
# reason break and not be able to install new stuff on the image.
# TODO: check above info, no issues yet (2024-12-11)

#TODO: get this as a parameter
MIRTE_SRC_DIR=/usr/local/src/mirte
. tools.sh
# shellcheck source=/dev/null
source /etc/os-release

# Install ROS 2 (Humble)
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
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
sudo apt install -y ros-dev-tools

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

# if mirte-ros-packages is from main or develop, use the precompiled version, otherwise compile on-device
cd $MIRTE_SRC_DIR/mirte-ros-packages
git submodule update --init --recursive
branch=$(git rev-parse --abbrev-ref HEAD)
arch=$(dpkg --print-architecture)
ubuntu_version=$(lsb_release -cs)
github_url=$(git config --get remote.origin.url | sed 's/\.git$//')
fallback=true
if [[ $branch == "develop" || $branch == "main" ]]; then
	fallback=false

	# Install mirte ros packages with apt from github, since they take ages to compile and it's easier to update them.
	# colcon ignore those packages

	echo "Using precompiled version of mirte-ros-packages"
	cd /home/mirte/mirte_ws/src/mirte-ros-packages || exit 1
	ignore=(mirte_telemetrix_cpp mirte_msgs mirte_teleop) # mirte_control/mirte_master_base_control mirte_control/mirte_master_arm_control mirte_control/mirte_pioneer_control # TODO: this doesn't work with subfolders
	packages=''
	for i in "${ignore[@]}"; do
		touch $i/COLCON_IGNORE
		i_dash=$(echo $i | tr '_' '-')
		packages="$packages ros-$ROS_NAME-$i_dash"
	done
	if [[ $branch == "develop" ]]; then
		arch="${arch}_develop"
	fi
	echo "deb [trusted=yes] $github_url/raw/ros_mirte_${ROS_NAME}_${ubuntu_version}_${arch}/ ./" | sudo tee /etc/apt/sources.list.d/mirte-ros-packages.list
	echo "yaml $github_url/raw/ros_mirte_${ROS_NAME}_${ubuntu_version}_${arch}/local.yaml ${ROS_NAME}" | sudo tee /etc/ros/rosdep/sources.list.d/mirte-ros-packages.list
	sudo apt update
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
git clone https://github.com/RobotWebTools/web_video_server.git -b ros2
cd .. || exit 1
rosdep install -y --from-paths src/ --ignore-src --rosdistro $ROS_NAME
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
add_mirte_settings "export MIRTE_ZENOH=false" # disable zenoh by default, but can be enabled by setting to true
add_mirte_settings "export MIRTE_USE_MULTIROBOT=false" # TODO: use this in the launch file?
add_mirte_settings "export ROS_LOG_DIR=/tmp/ros"
add_rc "source /home/mirte/.mirte_settings.sh"
add_rc "# Enable Zenoh and multirobot in .mirte_settings.sh"

add_rc "source /home/mirte/mirte_ws/install/setup.bash" "source /home/mirte/mirte_ws/install/setup.zsh"
add_profile "export ROS_LOG_DIR=/tmp/ros_log/" # log to tmp to not fill up the disk
# shellcheck source=/dev/null
source /home/mirte/mirte_ws/install/setup.bash

# Add systemd service to start ROS nodes
if [[ $MIRTE_TYPE == "mirte-master" ]]; then # master version should start a different launch file
	# rename the service file to the correct name, otherwise systemctl will error with a "Failed to look up unit file state: Link has been severed" error
	mv $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-ros.service $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-ros-pioneer.service
	mv $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-master-ros.service $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-ros.service
fi
sudo rm /lib/systemd/system/mirte-ros.service || true
# uses same service name, but different links. The service file starts mirte_ros with the correct launch file as argument
sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-ros.service /lib/systemd/system/mirte-ros.service
sudo systemctl daemon-reload
sudo systemctl stop mirte-ros || /bin/true
sudo systemctl start mirte-ros
sudo systemctl enable mirte-ros

sudo usermod -a -G video mirte
sudo adduser mirte dialout

# Some nice extra packages: clean can clean workspaces and packages. No need to do it by hand. lint can check for errors in the cmake/package code.
sudo pip3 install colcon-clean colcon-lint

# Add colcon top level workspace, this makes it possible to run colcon build from any folder, it will find the workspace and build it. Otherwise it will create a new workspace in the subdirectory.
cd /tmp
git clone https://github.com/rhaschke/colcon-top-level-workspace
cd colcon-top-level-workspace
pip install .
cd ..
rm -rf colcon-top-level-workspace
if [[ $MIRTE_TYPE == "mirte-master" ]]; then
	# TODO: need to check and edit the next part:
	sudo apt install ros-$ROS_NAME-slam-toolbox -y

	# install lidar and depth camera
	cd /home/mirte/mirte_ws/src || exit 1
	git clone https://github.com/Slamtec/rplidar_ros.git -b ros2 # FIXME-FUTURE: Can be installed in newer versions if V2.1.5 is released

	git clone https://github.com/ArendJan/ros2_astra_camera.git -b fix-ros-jammy      # compressed images image transport fixes, fork of orbbec/... with also lazy nodes
	git clone https://github.com/clearpathrobotics/clearpath_mecanum_drive_controller # FIXME: Can be installed from apt? why build?
	cd ../../
	mkdir temp
	cd temp || exit 1
	sudo apt install -y libudev-dev libusb-1.0-0-dev nlohmann-json3-dev
	# Install lubuvc-dev manually for newer version
	git clone https://github.com/libuvc/libuvc.git
	cd libuvc
	mkdir build && cd build
	cmake .. && make -j4
	sudo make install
	sudo ldconfig
	cd ../../../
	sudo rm -rf temp
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
eval "\$(register-python-argcomplete3 ros2)"
eval "\$(register-python-argcomplete3 colcon)"
EOF
fi
