#!/bin/bash

# This script creates USB gadgets usign ConfigFS for both Linux/MacOS and Windows
# The Linux and MacOS version will connect to usb0, whil Windows will connect
# to usb1. Both networks then are shown on the host.
MIRTE_SRC_DIR=/usr/local/src/mirte

rmmod xradio_wlan || true # fix for orange pi zero
modprobe libcomposite
sudo killall -9 dnsmasq
sudo $MIRTE_SRC_DIR/mirte-install-scripts/ev3-usb.sh down "$(ls /sys/class/udc | tail -n1)" || true
sudo $MIRTE_SRC_DIR/mirte-install-scripts/ev3-usb.sh up "$(ls /sys/class/udc | tail -n1)" || true

function setup_network_usb() {
	USB_NAME=$1
	SUBNET=$2
	echo "setup $USB_NAME on 192.168.$SUBNET.xxx"
	sudo ip address add 192.168."$SUBNET".1/24 dev "$USB_NAME"
	sudo ip link set up "$USB_NAME"
	sudo sysctl -w net.ipv4.ip_forward=1

	sudo iptables -F
	sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
	sudo iptables -A FORWARD -i wlan0 -o "$USB_NAME" -m state --state RELATED,ESTABLISHED -j ACCEPT
	sudo iptables -A FORWARD -i "$USB_NAME" -o wlan0 -j ACCEPT

	sudo dnsmasq --address=/#/192.168."$SUBNET".1 --dhcp-range=192.168."$SUBNET".10,192.168."$SUBNET".100 --conf-file --domain-needed --bogus-priv --server=8.8.8.8 --dhcp-option=option:dns-server,8.8.8.8 --interface="$USB_NAME" --except-interface=lo --bind-interfaces
}

#start handing out subnets from 43
SUBNET=43

# prefer usb1 to have .43.xxx(orange pi zero2 uses this for windows)
if [ -d /sys/class/net/usb1 ]; then
	setup_network_usb usb1 $SUBNET
	((SUBNET += 1))
fi

# Loop over rest of usb interfaces to setup as well.
for USB_PATH in /sys/class/net/usb*; do
	USB=$(basename $USB_PATH)
	if [[ $USB == "usb1" ]]; then
		continue
	fi
	setup_network_usb $USB $SUBNET
	((SUBNET += 1))
done

modprobe xradio_wlan || true # fix for orange pi zero