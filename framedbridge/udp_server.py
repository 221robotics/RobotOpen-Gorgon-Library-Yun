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


# setup the serial port
usbPort = '/dev/ttyATH0'
sp = serial.Serial(usbPort, 115200, timeout=1)


# framedbridge callback
def onDecodedFrameCallback(frame):
    # make sure a dashboard has connected and sent us data
    if recvPort != 0:
        # send the robotopen packet to the DS
        sock.sendto(frame, (recvIp, recvPort))

# framedbridge instance
fb = FramedBridge(onDecodedFrameCallback)


# used to split packets into smaller chunks
# http://code.activestate.com/recipes/425397-split-a-list-into-roughly-equal-sized-pieces/
def split_seq(seq, size):
        newseq = []
        splitsize = 1.0/size*len(seq)
        for i in range(size):
                newseq.append(seq[int(round(i*splitsize)):int(round((i+1)*splitsize))])
        return newseq


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
        splitFrames = split_seq(encodedFrame, 2)

        # write the framedbridge packet (first half) to the atmega
        sp.write(''.join(chr(b) for b in splitFrames[0]))

        # give the arduino a short break to catch up (small serial buffer)
        time.sleep(0.006)

        # write the framedbridge packet (second half) to the atmega
        sp.write(''.join(chr(b) for b in splitFrames[1]))

    # read from serial
    while (sp.inWaiting() > 0):
        # read a single character (we don't want to block)
        dataIn = sp.read()

        # decode FramedBridge frame into robotopen packet
        fb.decode(bytearray(dataIn))


    # don't kill the CPU
    time.sleep(0.001)
