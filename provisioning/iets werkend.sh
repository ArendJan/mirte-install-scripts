#!/bin/bash -x

modprobe libcomposite

cd /sys/kernel/config/usb_gadget/
mkdir g || true
cd g

echo 0x1d6b > idVendor  # Linux Foundation
echo 0x0104 > idProduct # Multifunction Composite Gadget
echo 0x0100 > bcdDevice # v1.0.0
echo 0x0200 > bcdUSB    # USB 2.0

echo 0xEF > bDeviceClass
echo 0x02 > bDeviceSubClass
echo 0x01 > bDeviceProtocol

mkdir -p strings/0x409
echo "deadbeef00115599" > strings/0x409/serialnumber
echo "irq5 labs"        > strings/0x409/manufacturer
echo "Pi Zero Gadget"   > strings/0x409/product

# mkdir -p functions/acm.usb0    # serial
mkdir -p functions/rndis.usb0  # network

mkdir -p configs/c.1
echo 250 > configs/c.1/MaxPower

ln -s functions/rndis.usb0 configs/c.1/
# ln -s functions/acm.usb0   configs/c.1/

# OS descriptors
echo 1       > os_desc/use
echo 0xcd    > os_desc/b_vendor_code
echo MSFT100 > os_desc/qw_sign

echo RNDIS   > functions/rndis.usb0/os_desc/interface.rndis/compatible_id
echo 5162001 > functions/rndis.usb0/os_desc/interface.rndis/sub_compatible_id

ln -s configs/c.1 os_desc



# ln -s configs/c.2 os_desc
attr="0xC0" # Self powered
pwr="1"     # 2mA
cfg1="mass storage"
C=1
N="usb0"
g=.
mkdir  configs/c.$C || true
echo "${attr}" > configs/c.$C/bmAttributes
echo "${pwr}" > configs/c.$C/MaxPower
mkdir  configs/c.$C/strings/0x409 || true
echo "${cfg2}" > configs/c.$C/strings/0x409/configuration

mkdir -p  functions/mass_storage.$N || true
FILE=/root/test.img
mkdir -p ${FILE/img/d} || true
mount -t vfat -o rw $FILE ${FILE/img/d} || true
sleep 5

echo 0 > functions/mass_storage.$N/stall
echo 0 > functions/mass_storage.$N/lun.0/cdrom
echo 0 > functions/mass_storage.$N/lun.0/ro
echo 0 > functions/mass_storage.$N/lun.0/nofua
echo $FILE > functions/mass_storage.$N/lun.0/file
ln -s  functions/mass_storage.$N  configs/c.$C/



udevadm settle -t 5 || :
ls /sys/class/udc/ > UDC