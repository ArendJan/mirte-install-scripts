#!/bin/bash
set -xe
#TODO: fix for ROS2 if required

rm -rf ~/arm_temp || true
mkdir ~/arm_temp
cd ~/arm_temp
sudo systemctl stop mirte-ros
wget https://mirte.arend-jan.com/files/telemetrix/release/Telemetrix4RpiPico.uf2
sudo picotool info Telemetrix4RpiPico.uf2
sudo picotool load -f Telemetrix4RpiPico.uf2
cd /usr/local/src/mirte/mirte-tmx-pico-aio
git pull
pip install .
pip install aioconsole
python3 examples/mirte_master_reset_offset.py
sleep 10
python3 examples/mirte_master_set_ranges_volt.py
sleep 10
python3 examples/mirte_master_check_home.py </dev/tty # tty required as this script is piped
cd ~
rm -rf ~/arm_temp
cd ~/mirte_ws/src/mirte-ros-packages/
git pull
catkin build
sudo systemctl start mirte-ros
