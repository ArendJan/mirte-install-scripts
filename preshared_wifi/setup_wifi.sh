#!/bin/bash

# set -xe

seed="asdf"
mac=$(ip addr show "wlan0" | awk '/ether/{print $2}')
ssid="TUD-facility"
# for file in $ssid/ try to decrypt
for file in $ssid/*.enc; do
	# decrypt file
	(openssl enc -d -nosalt -aes256 -k "pass:$seed$mac" -in "$file" -out "$ssid/found.txt" >/dev/null 2>&1) && found=true || true
	if [ "$found" = true ]; then
		# get password
		password=$(cat "$ssid/found.txt")
		curr_netw=$(nmcli -t -f NAME c show --active)

		nmcli c down "$curr_netw" || true
		nmcli c delete "$ssid" || true
		sudo nmcli con add con-name "$ssid" type wifi ssid "$ssid" wifi-sec.key-mgmt wpa-psk wifi-sec.psk $password
		nmcli c up "$ssid"
		echo "Found password for $ssid, should be connected now!"
		rm "$ssid/found.txt"
		break
	fi
done
