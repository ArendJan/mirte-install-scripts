#!/bin/bash

set -xe

seed="asdf"
mac=$(ip addr show "$(awk 'NR==3{print $1}' /proc/net/wireless | tr -d :)" | awk '/ether/{print $2}')
ssid="xxxxxx"
# for file in $ssid/ try to decrypt
for file in $ssid/*.enc; do
	# decrypt file
	openssl enc -d -aes256 -k "pass:$seed$mac" -in "$ssid/$file" -out "$ssid/found.txt" && found=true || true
	if [ "$found" = true ]; then
		# get password
		password=$(cat "$ssid/found.txt")
		echo "Password found: $password"
		curr_netw=$(nmcli -t -f NAME c show --active)

		nmcli c down "$curr_netw" || true
		sudo nmcli con add con-name "$ssid" type wifi ssid "$ssid" wifi-sec.key-mgmt wpa-psk wifi-sec.psk $password
		break
	fi
done
