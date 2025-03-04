#!/bin/bash

function start_avahi {
	# Restart avahi-daemon, to clear all previous addresses and hosts
	service avahi-daemon restart && sleep 1

	# Publish avahi (not using daemon since we publish two addresses)
	avahi-publish-address -R mirte.local "$(hostname -I | awk '{print $1}')" &
	avahi-set-host-name "$(cat /etc/hostname)"
	avahi-publish-service "$(cat /etc/hostname)" _mirte._tcp 80 &
	avahi-publish-service "$(cat /etc/hostname)" _arduino._tcp 80 &
}

# This function basically just starts wifi-connect
# only taking some limitations into account
function start_acces_point {
	# remove own connection from nm and killall
	rm -rf /etc/NetworkManager/system-connections/"$(cat /etc/hostname)"*
	sudo killall -9 wifi-connect || /bin/true
	sudo killall -9 blink.sh || /bin/true
	echo "Killed all previous instances"

	# It takes some time for NetworkManager to find all
	# networks.
	nmcli con down "$(cat /etc/hostname)"
	iw dev wlan0 scan | grep SSID
	nmcli device wifi list
	echo "Rescanned networks"

	# Start wifi-connect (this starts the AP, and uses dnsmasq
	# as DHCP server
	wifi-connect -o 8080 -p "$(cat /home/mirte/.wifi_pwd)" -s "$(cat /etc/hostname)" &

	# Wait until the AP is up
	until [ -f /etc/NetworkManager/system-connections/"$(cat /etc/hostname)".nmconnection ]; do
		sleep .1
		echo " waiting for network"
	done

	# And modify the network in a way that avahi mdns packages will
	# get through
	nmcli con modify "$(cat /etc/hostname)" 802-11-wireless-security.proto wpa
	#    nmcli con modify `cat /etc/hostname` 802-11-wireless-security.group ccmp
	#    nmcli con modify `cat /etc/hostname` 802-11-wireless-security.pairwise ccmp
	# generate a random channel, can even change to 5Ghz if wanted, then check how to find a correct channel
	nmcli c modify "$(cat /etc/hostname)" 802-11-wireless.band bg 802-11-wireless.channel "$(shuf -i 1-11 -n 1)"
	nmcli con down "$(cat /etc/hostname)"
	sleep 10
	nmcli con up "$(cat /etc/hostname)"

	# Start all avahi addresses and services
	start_avahi

	# Blink ssid-ID
	UNIQUE_ID=$(cat /etc/hostname | cut -c7-12)
	$MIRTE_SRC_DIR/mirte-install-scripts/blink.sh $UNIQUE_ID &
}

function check_connection {
	# Remove my own networks
	sudo rm /etc/NetworkManager/system-connections/"$(cat /etc/hostname)".*

	# Only look for networks if you already connected to one
	nr=$(ls /etc/NetworkManager/system-connections/ | wc -l)
	if [ "$nr" -gt 0 ]; then
		# Wait for a connection with a known ssid (timeout 10 seconds)
		nmcli device set wlan0 autoconnect yes
		TIMEOUT=25
		NEXT_WAIT_TIME=0
		until [ $NEXT_WAIT_TIME -eq $TIMEOUT ] || sudo nmcli con show --active | grep wlan0; do
			echo "waiting for connection"
			sleep 1
			let "NEXT_WAIT_TIME=NEXT_WAIT_TIME+1"
		done
	fi

	# Get wifi connection if connected
	if sudo nmcli con show --active | grep wlan0; then
		# Bugfix (see network_install.sh)
		# sudo ln -s /run/systemd/resolve/resolv.conf /etc/resolv.conf

		printf 'Connected to wifi connection:'
		nmcli con show --active | grep wlan0
		$MIRTE_SRC_DIR/mirte-install-scripts/blink.sh "$(hostname -I)" &
		start_avahi
	else
		printf 'No connection found, starting AP with wifi connect\n'
		start_acces_point

		# Restart the whole network process when connection did not take place
		while inotifywait -e modify /etc/NetworkManager/system-connections/"$(cat /etc/hostname)".nmconnection; do echo "hoi"; done
		# TODO: check: mybe this can be enabled again (currently this does not make thesaved wifi peristent
		#      printf "Networkmanager settings changed, restarting wifi-connect\n"
		#      sleep 5 # Give wifi-connect the possibility to change the settings
		#      sudo killall -9 wifi-connect || /bin/true
		#      nmcli con down `cat /etc/hostname`
		#      iw dev wlan0 scan | grep SSID
		#      nmcli device wifi list
		#      echo "Rescanned networks"
		#      printf "And doing the next thing\n"
		#      check_connection
	fi
}

function check_ssh_host_keys() {
	# Sometimes these files are empty after first boot and ssh won't work.
	# This will check the files for size and regenerate them when one of them is empty
	# Similar issues: https://github.com/NixOS/nixpkgs/issues/98842 and https://forum.armbian.com/topic/12934-ssh-host-keys-not-generated/
	if
		file_empty "/etc/ssh/ssh_host_ecdsa_key" || file_empty "/etc/ssh/ssh_host_ecdsa_key.pub" ||
			file_empty "/etc/ssh/ssh_host_ed25519_key" || file_empty "/etc/ssh/ssh_host_ed25519_key.pub" ||
			file_empty "/etc/ssh/ssh_host_rsa_key" || file_empty "/etc/ssh/ssh_host_rsa_key.pub"
	then
		echo "Regenerating ssh host keys and restarting sshd"
		ssh-keygen -A
		systemctl restart sshd
	fi

}
function file_empty() {
	FILE=$1
	if [[ ! -s $FILE ]]; then
		echo $FILE is empty
		return 0 # true
	fi
	return 1 # false

}

MIRTE_SRC_DIR=/usr/local/src/mirte

# Create unique SSID
# This must be run every time on boot, since it should
# be generated on first boot (so not when generating
# the image in network_setup.sh)

if false; then # when using mac for hostname.
	mac=$(ip addr show "wlan0" | awk '/ether/{print $2}')
	UNIQUE_ID=$(echo -n $mac | tr -cd "1-9A-Fa-f" | tail -c 6) # last 6 characters of mac address, without colons or 0s
	MIRTE_SSID="Mirte-$(echo ${UNIQUE_ID^^})"
	echo "Generated SSID: $MIRTE_SSID"
	# add to check: || [[ "$(cat /etc/hostname)" != "$MIRTE_SSID" ]]
fi

if [ ! -f /etc/ssid ] || [[ $(cat /etc/hostname) == "Mirte-XXXXXX" ]]; then
	UNIQUE_ID=$(tr -cd "1-9A-F" </dev/urandom | head -c 6)
	MIRTE_SSID=Mirte-$(echo ${UNIQUE_ID^^})
	sudo bash -c 'echo '$MIRTE_SSID' > /etc/hostname'
	sudo ln -s /etc/hostname /etc/ssid
	# And add them to the hosts file
	sudo bash -c 'echo 127.0.0.1 '$MIRTE_SSID' >> /etc/hosts'
	sudo bash -c 'echo "127.0.0.1 mirte" >> /etc/hosts'
	# Not needed anymore once we first generate the SSID and then start the network
	sudo bash -c 'echo "127.0.0.1 Mirte-XXXXXX" >> /etc/hosts'
fi

check_ssh_host_keys

$MIRTE_SRC_DIR/mirte-install-scripts/usb_ethernet.sh

check_connection

# This should run forever, otherwise systemd will shut it down
sleep infinity
