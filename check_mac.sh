#!/bin/bash

set -xe

local_folder=$(dirname "$0")

wifi_dev="wlan0"

mac=$(ip addr show "$wifi_dev" | awk '/ether/{print $2}')
echo "MAC: $mac"
# if mac is not in macs.txt, print error and exit
if ! grep -q "$mac" "$local_folder/macs.txt"; then
    echo "De MAC van je Orange Pi is niet geregistreerd. Meld dit op Brightspace: https://brightspace.tudelft.nl/d2l/le/683193/discussions/topics/94526/View met vermelding van dit MAC-adres: ${mac}."
    # exit 1
else 
    echo "MAC is al geregistreerd, je hoeft niks te doen!"
fi