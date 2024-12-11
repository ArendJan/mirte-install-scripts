#!/bin/bash

# some functions to help with the setup

add_rc() {
	lines=$1

	grep -qxF "$1" ~/.bashrc || echo "$1" >>~/.bashrc

	# if we have a second argument, we use that as lines to add to zshrc, otherwise keep the same as bashrc
	if [ -z "$2" ]; then
		lines=$1
	else
		lines=$2
	fi
	grep -qxF "$1" ~/.zshrc || echo "$1" >>~/.zshrc
}
