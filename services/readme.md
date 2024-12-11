# Services

List of services:
- mirte-ap:
  - start wifi hotspot at boot
- mirte-jupyter:
  - starts jupyter, not on by default
- mirte-ros
  - starts ROS using mirte_ros.sh
    - this tries to guess the IP of the robot by using hostname -I and uses this as ROS_IP
    - 
- mirte-shutdown
  - Only for Mirte-master
  - Will run mirte_boot.sh and mirte_shutdown.sh
  - boot:
    - check if last shutdown was correct (does ~/.shutdown exist) and write failures to ~/.shutdown_incorrect
    - starts mirte_master_check.sh
      - checks for /mirte/power/power_watcher::percentage
      - if not exists for too long(15min): shutdown
  - shutdown:
    - writes the ~/.shutdown file that is checked at boot
    - Prints "Shutting down....." to the oled screen by calling the rosservice
- mirte-usb-switch:
  - starts the usb-switch to turn on the orbbec camera
- mirte-web-interface:
  - starts the web interface
- mirte-wifi-watchdog:
  - TODO: not sure