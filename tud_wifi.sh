#!/bin/bash

ssid="TUD-facility"
# ssid="RoboHouse Gasten"
wifi_dev="wlan0"
# wifi_dev="wlp0s20f3"
echo "$ssid Wi-Fi Setup"
set -e

trim() {
	local trimmed="$1"

	# Strip leading space.
	trimmed="${trimmed## }"
	# Strip trailing space.
	trimmed="${trimmed%% }"

	echo "$trimmed"
}

# curr_netw="$(iwgetid -r)" || true # doesn't work for hotspots
curr_netw="$(iw dev wlan0 info | grep ssid | awk '!($1="")')" || true # todo: fix for spaces in front
curr_netw=$(trim "$curr_netw")
# if already connected to $ssid, show ip and done
if [ "$curr_netw" == "$ssid" ]; then
	echo "Already connected to $ssid"
	echo "IP: $(ip addr show $wifi_dev | grep -Po 'inet \K[\d.]+')"
	exit 0
fi

# show mac for online portal
mac=$(ip addr show "$wifi_dev" | awk '/ether/{print $2}')
echo "MAC (for registering online): $mac"

# wait for wifi password input
echo "Please enter the password for $ssid:"
read password

# if curr_netw is not empty, set it down
if [ -n "$curr_netw" ]; then
	nmcli c down "$curr_netw" || true
fi

nmcli d wifi rescan || true
sleep 10

# Check if the desired SSID is in the list of available networks
if ! nmcli d wifi list | grep -q "$ssid"; then
	echo "$ssid not found in the list of available networks."
	if [ -n "$curr_netw" ]; then
		echo "Reconnecting to the previous network: $curr_netw"
		nmcli c up "$curr_netw" || true
	fi
	exit 1
fi

nmcli c delete "$ssid" || true # delete old connection

if ! sudo nmcli d wifi c "$ssid" password "$password"; then
	echo "Unable to connect to $ssid, check the MAC and password and try again."
	exit 1
fi

if [ "$(iwgetid -r)" != "$ssid" ]; then
	echo "Failed to connect to $ssid, check MAC and password and retry."
	exit 1
fi

echo "Connected to $ssid!"
echo "IP: $(ip addr show "$wifi_dev" | grep -Po 'inet \K[\d.]+')"

exit 0
