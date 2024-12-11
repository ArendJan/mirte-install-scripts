#!/bin/bash

set -xe

salt="asdf"
# for line in wifipassword.csv, create file with password and encrypt with seed+mac
ssid="asdf"
mkdir -p "$ssid"
currdir=$(pwd)
count=1
for line in $(cat wifipassword.csv); do
    mac=$(echo $line | cut -d, -f1)
    password=$(echo $line | cut -d, -f2)
    # use wpa_passphrase to get the hashed password
    psk=$(wpa_passphrase "$ssid" "$password" | grep "psk" | grep -v "#" )
    eval $psk
    echo $psk > "$ssid/$count.txt"
    openssl enc -e -aes256 -k "pass:$salt$mac" -in "$currdir/$ssid/$count.txt" -out "$currdir/$ssid/$count.enc" 
    rm "$ssid/$count.txt"
    count=$((count+1))
done