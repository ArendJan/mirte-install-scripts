#!/bin/bash
set -xe
MIRTE_SRC_DIR=/usr/local/src/mirte
# Make sure there are no conflicting hcdp-servers
sudo apt install -y dnsmasq-base
systemctl disable hostapd
sed -i 's/#DNSStubListener=yes/DNSStubListener=no/g' /etc/systemd/resolved.conf
# Install netplan (not installed on armbian) and networmanager (not installed by Raspberry)
sudo apt install -y netplan.io
sudo apt install -y network-manager
sudo cp $MIRTE_SRC_DIR/mirte-install-scripts/50-cloud-init.yaml /etc/netplan/
sudo apt purge -y ifupdown

# Fix for bug in systemd-resolved
# (https://askubuntu.com/questions/973017/wrong-nameserver-set-by-resolvconf-and-networkmanager)
# For the installation we need 8.8.8.8, but linking will be done in network_setup.sh
sudo rm -rf /etc/resolv.conf
sudo bash -c 'echo "nameserver 8.8.8.8" > /etc/resolv.conf'
# Install wifi-connect
MY_ARCH=$(arch)
if [[ $MY_ARCH == "armv7l" ]]; then MY_ARCH="armv7hf"; fi
wget https://github.com/balena-os/wifi-connect/releases/download/v4.11.1/wifi-connect-v4.11.1-linux-"$(echo "$MY_ARCH")".zip
unzip -h || sudo apt install -y unzip
unzip wifi-connect*
sudo mv wifi-connect /usr/local/sbin
rm wifi-connect*
# Added systemd service to account for fix: https://askubuntu.com/questions/472794/hostapd-error-nl80211-could-not-configure-driver-mode
sudo rm /lib/systemd/system/mirte-ap.service || true
sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-ap.service /lib/systemd/system/

sudo systemctl daemon-reload
sudo systemctl stop mirte-ap || /bin/true
sudo systemctl start mirte-ap
sudo systemctl enable mirte-ap
# Added systemd service to check on boot error for OPi
sudo rm /lib/systemd/system/mirte-wifi-watchdog.service || true
sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-wifi-watchdog.service /lib/systemd/system/

sudo systemctl daemon-reload
sudo systemctl stop mirte-wifi-watchdog || /bin/true
sudo systemctl start mirte-wifi-watchdog
sudo systemctl enable mirte-wifi-watchdog
# Install avahi
sudo apt install -y libnss-mdns
sudo apt install -y avahi-utils avahi-daemon
sudo apt install -y avahi-utils avahi-daemon # NOTE: Twice, since regular apt installation on armbian fails (https://forum.armbian.com/topic/10204-cant-install-avahi-on-armbian-while-building-custom-image/)
# Disable lo interface for avahi
sed -i 's/#deny-interfaces=eth1/deny-interfaces=lo/g' /etc/avahi/avahi-daemon.conf
# Install dependecies needed for setup script
sudo apt install -y inotify-tools wireless-tools
# Disable ssh root login
sed -i 's/#PermitRootLogin yes/PermitRootLogin no/g' /etc/ssh/sshd_config
# Install usb_ethernet script from EV3
wget https://raw.githubusercontent.com/ev3dev/ev3-systemd/ev3dev-buster/scripts/ev3-usb.sh -P $MIRTE_SRC_DIR/mirte-install-scripts
sudo chmod +x $MIRTE_SRC_DIR/mirte-install-scripts/ev3-usb.sh
sudo chown mirte:mirte $MIRTE_SRC_DIR/mirte-install-scripts/ev3-usb.sh
sudo bash -c 'echo "libcomposite" >> /etc/modules'
# remove g_serial from modules to let the ev3-usb script enable usb ethernet on the orange pi zero 1 as well.
sudo bash -c "sed -i '/g_serial/d' /etc/modules"

# Generate wifi password (TODO: generate random password and put on NTFS)
if [ ! -f /home/mirte/.wifi_pwd ]; then
	bash -c 'echo mirte_mirte > /home/mirte/.wifi_pwd'
fi

# Allow wifi_pwd to be modified using the web interface
sudo chmod 777 /home/mirte/.wifi_pwd

# Add hostname and make it writable
sudo bash -c 'echo "Mirte-XXXXXX" > /etc/hostname'
sudo chmod 777 /etc/hostname

# Fix for wpa_supplicant error
sudo bash -c "echo 'match-device=driver:wlan0' >> /etc/NetworkManager/NetworkManager.conf"

# Reboot after kernel panic
# The OPi has a fairly unstable wifi driver which might
# panic the kernel (at boot). Instead of waiting an unkown
# time and reboot manually, we will reboot automatically
sudo bash -c 'echo "kernel.panic = 10" > /etc/sysctl.conf'

# separate service for usb ethernet
sudo rm /lib/systemd/system/mirte-usb.service || true
sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-usb.service /lib/systemd/system/
sudo systemctl daemon-reload
sudo systemctl stop mirte-usb || /bin/true
sudo systemctl start mirte-usb
sudo systemctl enable mirte-usb
