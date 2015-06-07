# opkg update
# opkg install pyserial


import socket
import time
import serial
from select import select
from framedbridge import FramedBridge


# the last ip/port we received data from
recvIp = ''
recvPort = 0

# UDP server config
UDP_IP = "0.0.0.0"
UDP_PORT = 22211

# start UDP server
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# outgoing buffer vars
pendingTxBuffer = bytearray()
readyToSend = False


# setup the serial port
usbPort = '/dev/ttyATH0'
sp = serial.Serial(usbPort, 115200, timeout=1)


# framedbridge data ready callback
def onDecodedFrameCallback(frame):
    # make sure a dashboard has connected and sent us data
    if recvPort != 0:
        # send the robotopen packet to the DS
        sock.sendto(frame, (recvIp, recvPort))


# framedbridge flow control CTS callback
def onClearToSendCallback():
    readyToSend = True
    print 'time to xmit'


# framedbridge instance
fb = FramedBridge(onDecodedFrameCallback, onClearToSendCallback)






while True:
    # check if there is any data waiting via the UDP socket
    rd, wr, err = select([sock], [], [], 0)

    # read from socket
    if len(rd) > 0:
        data, addr = sock.recvfrom(1024) # buffer size is (up to) 1024 bytes

        # ip address of the DS
        recvIp = addr[0]

        # UDP port of the DS
        recvPort = addr[1]

        # encode UDP packet into FramedBridge frame and split into 2 pieces
        encodedFrame = fb.encode(bytearray(data))

        # if this is a heartbeat packet, send immediately (it's only 3 bytes)
        if chr(encodedFrame[0]) == 'h':
            sp.write(''.join(chr(b) for b in encodedFrame))

            # clear anything in the outgoing buffer (since the robot is now disabled)
            # ??? is this a problem if we are sending a setparameter packet while disabled?
            pendingTxBuffer = bytearray()
        else:
            # need to split up packet into 64 byte chunks, buffer the remaining bytes until we receive CTS
            pass

    # read from serial
    while (sp.inWaiting() > 0):
        # read a single character (we don't want to block)
        dataIn = sp.read()

        # decode FramedBridge frame into robotopen packet
        fb.decode(bytearray(dataIn))


    # don't kill the CPU
    time.sleep(0.001)
