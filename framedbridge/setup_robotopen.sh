#!/bin/bash

# RobotOpen Yun Setup Script v1.0 [03.09.15]
# Install via curl -sSLk https://raw.githubusercontent.com/221robotics/RobotOpen-Gorgon-Library-Yun/master/framedbridge/setup_robotopen.sh | sh

# disable arduino bridge
sed -i '/ttyATH0/ s?^?#?' /etc/inittab

# update package manager
opkg update
# install wget
opkg install wget
# install pyserial
opkg install pyserial

# make sure we're in the root dir
cd /root

# create framedbridge working dir
mkdir framedbridge

# change into framedbridge dir
cd framedbridge

# download framedbridge from github
wget --no-check-certificate https://raw.githubusercontent.com/221robotics/RobotOpen-Gorgon-Library-Yun/master/framedbridge/framedbridge.py
wget --no-check-certificate https://raw.githubusercontent.com/221robotics/RobotOpen-Gorgon-Library-Yun/master/framedbridge/udp_server.py

# start framedbridge on boot
sed -i '/exit 0/i python /root/framedbridge/udp_server.py' /etc/rc.local

# done, let's reboot this sucker
echo "Success! Please reboot your Yun to activate RobotOpen (reboot -f)"