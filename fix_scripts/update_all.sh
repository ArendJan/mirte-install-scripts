#!/bin/bash
set -xe
#TODO: fix for ROS2 if required

sudo systemctl stop mirte-ros
mkdir ~/update_all || true
cd ~/update_all
wget -N https://mirte.arend-jan.com/files/telemetrix/release/Telemetrix4RpiPico.uf2.sha256sum
wget -N https://mirte.arend-jan.com/files/telemetrix/release/Telemetrix4RpiPico.uf2
wget -N https://mirte.arend-jan.com/files/telemetrix/release/Telemetrix4RpiPico.bin.sha256sum
sudo picotool info Telemetrix4RpiPico.uf2
sha256sum -c Telemetrix4RpiPico.uf2.sha256sum || exit 1
sudo picotool load -f Telemetrix4RpiPico.uf2
sleep 2
sudo picotool save -f Telemetrix4RpiPico.bin
sha256sum -c Telemetrix4RpiPico.bin.sha256sum || exit 1
cd ~
rm -rf ~/update_all
cd /usr/local/src/mirte/mirte-tmx-pico-aio
git pull
pip install .
pip install aioconsole
cd ~/mirte_ws/src/mirte-ros-packages/
git stash # save students changes
git pull
git stash pop || true # apply students changes
git status
echo "Resolve any merge conflicts before reboot!!!" # resolve merge conflicts before reboot
catkin build

cd /usr/local/src/mirte/mirte-install-scripts/
git pull
sudo ln -s /usr/local/src/mirte/mirte-install-scripts/services/mirte-shutdown.service /etc/systemd/system/mirte-shutdown.service || true
sudo touch /home/mirte/.shutdown
sudo systemctl enable --now mirte-shutdown
sudo systemctl daemon-reload

# uboot function
uboot() {
	cd ~
	mkdir uboot_fix/
	cd uboot_fix/
	# update u-boot to fix audio jack issue
	wget https://mirte.arend-jan.com/files/fixes/uboot/linux-u-boot-orangepi3b-edge_24.2.1_arm64__2023.10-S095b-P0000-H264e-V49ed-B11a8-R448a.deb
	sudo apt install ./linux-u-boot-orangepi3b-edge_24.2.1_arm64__2023.10-S095b-P0000-H264e-V49ed-B11a8-R448a.deb
	sudo bash -c 'source /usr/lib/u-boot/platform_install.sh; write_uboot_platform_mtd $DIR /dev/mtdblock0'
	cd ../
	rm -rf uboot_fix/
}

# if linux-u-boot-orangepi3b-edge is not installed, install it
apt -qq list linux-u-boot-orangepi3b-edge | grep 24 || uboot

sudo systemctl start mirte-ros
sudo touch /forcefsck
sync
# find any merge conflicts by looking for '<<<' in the code
grep -r '<<<' ~/mirte_ws/src/mirte-ros-packages && echo "merge conflicts in mirte-ros-packages" || echo "no merge conflict"
