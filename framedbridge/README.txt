STEP ONE
--------------
Use PuTTY to SSH into the Yun's IP address. Username: root, Password: arduino


STEP ONE
--------------
$ nano /etc/inittab

In the inittab file, comment out the line beginning with ttyATH0:

# ttyATH0::askfirst:/bin/ash --login


STEP TWO
--------------
Copy this framedbridge folder to /root (WinSCP can be used for this with the same login details used for PuTTY)


STEP THREE
--------------
Add this to the end of /etc/rc.local before exit 0:
python /root/framedbridge/udp_server.py


STEP FOUR
--------------
$ opkg update
$ opkg install pyserial

$ reboot -f
