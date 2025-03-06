#!/bin/bash

# some functions to help with the setup

add_rc() {
	lines=$1

	grep -qxF "$lines" ~/.bashrc || echo "$lines" >>~/.bashrc

	# if we have a second argument, we use that as lines to add to zshrc, otherwise keep the same as bashrc
	if [ -z "$2" ]; then
		lines=$1
	else
		lines=$2
	fi
	grep -qxF "$lines" ~/.zshrc || echo "$lines" >>~/.zshrc
}

add_profile() {
	lines=$1

	grep -qxF "$lines" ~/.profile || sed -i "10i\\$lines\\" ~/.profile
}

add_mirte_settings() {
	lines=$1
	echo "$lines" >>~/.mirte_settings.sh
}
