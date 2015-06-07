#!/bin/bash

# RobotOpen Yun Setup Script v1.0 [03.09.15]
# Install via curl -sSLk https://raw.githubusercontent.com/221robotics/RobotOpen-Gorgon-Library-Yun/flowcontrol/framedbridge/setup_robotopen.sh | sh

# disable arduino bridge
sed -i '/ttyATH0::askfirst:/c\#ttyATH0::askfirst:/bin/ash --login' /etc/inittab

# update package manager
opkg update

# install wget
opkg install wget

# install pyserial
opkg install pyserial

# make sure we're in the root dir
cd /root

# remove old framedbridge dir (if it exists)
rm -rf framedbridge

# create framedbridge working dir
mkdir framedbridge

# change into framedbridge dir
cd framedbridge

# download framedbridge from github
wget --no-check-certificate https://raw.githubusercontent.com/221robotics/RobotOpen-Gorgon-Library-Yun/flowcontrol/framedbridge/framedbridge.py
wget --no-check-certificate https://raw.githubusercontent.com/221robotics/RobotOpen-Gorgon-Library-Yun/flowcontrol/framedbridge/udp_server.py

# start framedbridge on boot
if grep -Fq "udp_server" /etc/rc.local
then
    # found line, no need to add anything
    echo "udp_server already set to start on boot"
else
    # did not find line, need to add it
    sed -i '/exit 0/i python /root/framedbridge/udp_server.py' /etc/rc.local
fi

# read EEPROM off chip when the Yun is being programmed via the network
if grep -Fq "eeprom:r:" /usr/bin/run-avrdude
then
    # found line, no need to add anything
    echo "EEPROM is already set to be read on code upload"
else
    # did not find line, need to add it
    sed -i 's+.*avrdude.*+avrdude -c linuxgpio -C /etc/avrdude.conf -p m32u4 -U eeprom:r:/tmp/eeprom.bin:r\n&+' /usr/bin/run-avrdude
fi

# ensure that we always write the contents of the EEPROM back to the chip when the Yun is being programmed via the network
sed -i '/lfuse:w:0xFF:m/c\avrdude -c linuxgpio -C /etc/avrdude.conf -p m32u4 -U lfuse:w:0xFF:m -U hfuse:w:0xD8:m -U efuse:w:0xFB:m -Uflash:w:$1:i -U eeprom:w:/tmp/eeprom.bin $2' /usr/bin/run-avrdude

# done, let's reboot this sucker
echo "Success! Please reboot your Yun to activate RobotOpen (reboot -f)"
