#!/bin/bash

BLINK_SPEED=.5
VALUE=$1

findleds() {
	LEDS_ALL=$(ls /sys/class/leds)
	# try to set green as first led, red as second
	# if not found, set the first two leds found
	if echo "$LEDS_ALL" | grep -q "green"; then
		GR_LED="$(echo "$LEDS_ALL" | grep "green")"
	elif echo "$LEDS_ALL" | grep -q "ACT"; then
		GR_LED="$(echo "$LEDS_ALL" | grep "ACT")"
	else
		GR_LED="$(echo "$LEDS_ALL" | head -n 1)"
	fi

	if echo "$LEDS_ALL" | grep -q "red"; then
		RED_LED="$(echo "$LEDS_ALL" | grep "red")"
	elif echo "$LEDS_ALL" | grep -q "PWR"; then
		RED_LED="$(echo "$LEDS_ALL" | grep "PWR")"
	else
		RED_LED="$(echo "$LEDS_ALL" | head -n 2 | tail -n 1)"
	fi

	echo "RED: $RED_LED"
	echo "GREEN: $GR_LED"
}
findleds
# store trigger to reset later to, take only part in [ ] to remove the list of options
# GR_TRIGGER="$(cat /sys/class/leds/$GR_LED/trigger | awk -F'[][]' '{print $2}')"
# RED_TRIGGER="$(cat /sys/class/leds/$RED_LED/trigger | awk -F'[][]' '{print $2}')"

echo "none" >/sys/class/leds/$GR_LED/trigger
echo "none" >/sys/class/leds/$RED_LED/trigger

red_on() {
	echo 'default-on' >/sys/class/leds/$RED_LED/trigger
}
red_off() {
	echo 'none' >/sys/class/leds/$RED_LED/trigger
}

green_on() {
	echo 'default-on' >/sys/class/leds/$GR_LED/trigger
}

green_off() {
	echo 'none' >/sys/class/leds/$GR_LED/trigger
}

echo "Blinking"
echo "$VALUE"

for ((repeat = 0; repeat < 5; repeat++)); do
	# Start sequence with fast blinking both
	for ((i = 0; i < 10; i++)); do
		FAST=$(echo "scale=2; $BLINK_SPEED/20" | bc)
		green_on
		red_on
		sleep "$FAST"
		green_off
		red_off
		sleep "$FAST"
	done
	sleep $BLINK_SPEED

	for ((i = 0; i < ${#VALUE}; i++)); do
		# Next character
		green_on
		sleep $BLINK_SPEED
		green_off
		sleep $BLINK_SPEED

		if [ "${VALUE:i:1}" == "." ]; then
			green_on
			red_on
			sleep $BLINK_SPEED
			green_off
			red_off
			sleep $BLINK_SPEED
		else
			if [ "${VALUE:i:1}" == "0" ]; then
				TWICE=$(echo "scale=2; $BLINK_SPEED*2" | bc)
				sleep "$TWICE"
			else
				# Convert hex to decimal number
				DEC_VAL=$(echo "obase=10; ibase=16; ${VALUE:i:1}" | bc)

				# Blink decimal number
				for ((j = 0; j < DEC_VAL; j++)); do
					red_on
					sleep $BLINK_SPEED
					red_off
					sleep $BLINK_SPEED
				done
			fi
		fi
	done
done

# Set to nice blinking, defaults are often heartbeat and off.
echo "activity" >/sys/class/leds/$GR_LED/trigger
echo "heartbeat" >/sys/class/leds/$RED_LED/trigger
