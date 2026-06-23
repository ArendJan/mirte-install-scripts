#!/bin/bash
set -xe

MIRTE_SRC_DIR=${MIRTE_SRC_DIR:-/usr/local/src/mirte}

if [[ ${type:=""} != "mirte_orangepi3b" ]]; then
	# Fix for wrong sound card
	sudo bash -c 'cat <<EOT >> /etc/asound.conf
defaults.pcm.card 1
defaults.ctl.card 1
EOT'

fi

cd $MIRTE_SRC_DIR/mirte-install-scripts/mirte-master/usb_switch/
sudo apt install libgpiod-dev -y
mkdir build
cd build
cmake ..
make -j

function add_service() {
	service_name=$1
	sudo rm -f /lib/systemd/system/$service_name
	sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/$service_name /lib/systemd/system/
	sudo systemctl daemon-reload
	sudo systemctl stop $service_name || /bin/true
	sudo systemctl start $service_name
	sudo systemctl enable $service_name
}

add_service mirte-battery-watcher.service # check that battery is not empty and shutdown if it is
add_service mirte-shutdown.service        # show a message on the screen when shutting down and trigger a shutdown of the robot
add_service mirte-usb-switch.service      # turn on/off depth cam usb port.

# create a gpio group and add mirte to it. This is needed to access the gpio ports, otherwise only sudo is allowed.
sudo groupadd gpiod
sudo usermod -a -G gpiod mirte
sudo echo '# udev rules for gpio port access through libgpiod
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpiod", MODE="0660"' | sudo tee /etc/udev/rules.d/60-gpiod.rules
pip install gpiod==1.5.4 # python3.8 version

# chatgpt node stuff for @chris-pek
pip install gtts playsound openai==0.28.0 sounddevice scipy SpeechRecognition soundfile transformers datasets pyyaml pydub Elevenlabs || true # some strange package versions
pip install numpy==1.23.1                                                                                                                     # python3.8 fix

cd ~/mirte_ws
source /opt/ros/humble/setup.bash
rosdep install -y --from-paths src/ --ignore-src --rosdistro humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
