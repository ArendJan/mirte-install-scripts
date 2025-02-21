#!/bin/bash

# This script is used to connect to a Wi-Fi network using nmcli.
# It will prompt the user for the password, restart the script with nohup and the password as arguments.
# This allows the script to be started from a wifi network that is disconnected from when the new network is connected.
# The script will reconnect to the original network if the connection to the new network fails.

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
local_folder=$(dirname "$0")

# curr_netw="$(iwgetid -r)" || true # doesn't work for hotspots
curr_netw="$(iw dev $wifi_dev info | grep ssid | awk '!($1="")')" || true
curr_netw=$(trim "$curr_netw")
# if already connected to $ssid, show ip and done
if [ "$curr_netw" == "$ssid" ]; then
	echo "Already connected to $ssid"
	echo "IP: $(ip addr show $wifi_dev | grep -Po 'inet \K[\d.]+')"
	sudo pkill -f "$local_folder/blink.sh" || true
	sudo "$local_folder/blink.sh" "$(ip addr show "$wifi_dev" | grep -Po 'inet \K[\d.]+')" &
	exit 0
fi

if [ "$1" == "" ]; then 
	ret=0
	$local_folder/check_mac.sh true || ret=$?
	# ask user that they checked it if exit code is 1
	if [ $ret -eq 1 ]; then
		echo "Did you post it on BrightSpace and got confirmation that it was configured for you?"
		read -p "Press enter to continue, ctrl-c to exit"
	fi
	# show mac for online portal
	mac=$(ip addr show "$wifi_dev" | awk '/ether/{print $2}')
	echo "MAC (for registering online): $mac"

	# wait for wifi password input
	echo "Please enter the password for $ssid:"
	read password

	# run rest of script in nohup
	nohup "$0" "$ssid" "$password" >/tmp/wifi.txt 2>&1 &
	nohup_pid=$!
	tail -f /tmp/wifi.txt &
	tail_pid=$!
	wait $nohup_pid
	kill $tail_pid
	exit 0
else
	ssid="$1"
	password="$2"

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
		nmcli c up "$curr_netw" || true
		exit 1
	fi

	if [ "$(iwgetid -r)" != "$ssid" ]; then
		echo "Failed to connect to $ssid, check MAC and password and retry."
		nmcli c up "$curr_netw" || true
		exit 1
	fi

	echo "Connected to $ssid!"
	echo "IP: $(ip addr show "$wifi_dev" | grep -Po 'inet \K[\d.]+')"
	sudo pkill "blink.sh" || true
	sudo "$local_folder/blink.sh" "$(ip addr show "$wifi_dev" | grep -Po 'inet \K[\d.]+')" &
	exit 0
fi
