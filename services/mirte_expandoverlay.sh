#!/bin/bash

# check if overlayfs is mounted
if grep -qs 'overlayroot' /proc/mounts; then
  echo "overlayroot is on."
else
    echo "overlayroot is off. Exiting."
    exit 1
fi

# check if /home/mirte/.overlay_expanded file exists
if [ -f /home/mirte/.overlay_expanded ]; then
  echo "Overlay is already expanded."
  exit 1
fi

# get partition of /media/root-rw
DISK_AND_PART=$(realpath /dev/disk/by-label/mirte_root) # always this name

# remove p1 from $DISK_AND_PART
DISK=$(echo $DISK_AND_PART | sed 's/p.//')

PART=$(echo $DISK_AND_PART | sed 's/.*p//')
cat << EOF | sudo parted ---pretend-input-tty "$DISK" resizepart
$PART
y
100%
EOF
sudo resize2fs "$DISK_AND_PART"

touch /home/mirte/.overlay_expanded # create file to indicate that overlay is expanded, this is stored on the overlay
