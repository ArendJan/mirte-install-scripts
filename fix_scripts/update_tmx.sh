#!/bin/bash
set -xe
#TODO: fix for ROS2 if required

mkdir temp_upd
cd temp_upd
wget https://mirte.arend-jan.com/files/telemetrix/modules2/Telemetrix4RpiPico.uf2
sudo picotool load -f Telemetrix4RpiPico.uf2
cd ../
rm -rf temp_upd
cd ~/mirte_ws/src/mirte-ros-packages/
git pull
