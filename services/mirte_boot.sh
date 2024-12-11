#!/bin/bash

set -xe
# check if file /home/mirte/shutdown exists
if [ -f /home/mirte/.shutdown ]; then
	# check if file /home/mirte/shutdown_done exists
	echo "correct shutdown"
	rm /home/mirte/.shutdown
else
	echo "incorrect shutdown"
	touch /home/mirte/.shutdown_incorrect
	# append date to shutdown_incorrect file
	date >>/home/mirte/.shutdown_incorrect
fi
"$(dirname "$0")/mirte_master_check.sh" &
