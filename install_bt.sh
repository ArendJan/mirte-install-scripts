#!/bin/bash
set -xe

install_fake_bt() {
	# Currently the kernel does not support cheap Chinese btusb clones
	# (see: https://bugzilla.kernel.org/show_bug.cgi?id=60824)
	# There is some work done though to start supporting these.

	# One has to disable autosuspend for these to work. This can be done
	# as a parameter, but could also be done by applying patch
	# https://github.com/torvalds/linux/commit/0671c0662383eefc272e107364cba7fe229dee44
	# (which is not in 5.10.21). Since the patch might limit the
	# number of supported dongles (with a proposed fix
	# https://gist.github.com/nevack/6b36b82d715dc025163d9e9124840a07#gistcomment-3815787)
	# we will use the param setting. Instead of applying the two patches.

	# But as we can see in /sys/module/btusb/parameters/enable_autosuspend, defaults
	# to N (and reset to Y) at the moment anyway. So no need to set this at the moment.
	#sudo bash -c 'echo "options btusb reset=1 enable_autosuspend=0" > /etc/modprobe.d/99-csr-bluetoothdongle.conf'

	# But as noted (https://gist.github.com/nevack/6b36b82d715dc025163d9e9124840a07#gistcomment-3817395)
	# a fix should also be applied to bluetooth.ko. This is still a WIP, so we go for the quick and dirty
	# solution, by just disabling it for all for now.
	# TODO: make this kernel version independant (or wait for this all to be in the kernel)

	# Install linux headers
	wget https://imola.armbian.com/apt/pool/main/l/linux-5.10.21-sunxi/linux-headers-current-sunxi_21.02.3_armhf.deb
	sudo dpkg -i linux-headers*.deb

	# Install source
	wget https://cdn.kernel.org/pub/linux/kernel/v5.x/linux-5.10.21.tar.xz
	xz -d -v linux-5.10.21.tar.xz
	tar xvf linux-5.10.21.tar linux-5.10.21/net/bluetooth
	cd linux-5.10.21/net/bluetooth || exit 1

	# Apply 'fix'
	sed -i 's/flt_type = HCI_FLT_CLEAR_ALL;/\/\/flt_type = HCI_FLT_CLEAR_ALL;/g' hci_core.c
	sed -i 's/hci_req_add(req, HCI_OP_SET_EVENT_FLT, 1, &flt_type);/\/\/hci_req_add(req, HCI_OP_SET_EVENT_FLT, 1, &flt_type);/g' hci_core.c

	# Build and install
	make -C /lib/modules/5.10.21-sunxi/build M=$PWD modules
	sudo cp bluetooth.ko /lib/modules/5.10.21-sunxi/kernel/net/bluetooth/bluetooth.ko
}

sudo apt install -y bluez joystick
if [ "$(uname -a | grep sunxi)" != "" ]; then
	# currently only supporting cheap USB dongles on OrangePi
	install_fake_bt
else
	# Fix for ps3 controller not connecting
	sudo sh -c "echo '#Fix for ps3 controllers not connecting\nClassicBondedOnly=false\n' >> /etc/bluetooth/input.conf"
fi
