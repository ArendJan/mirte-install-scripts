# Mirte-master usb switch
the orbbec camera must be unpowered at the boot of the OrangePi3b, otherwise it will pull 1A more and not work.
Together with the USB switcher pcb, this will turn on(and off) the camera after boot using libgpiod.
Pin is GPIO4_C3.