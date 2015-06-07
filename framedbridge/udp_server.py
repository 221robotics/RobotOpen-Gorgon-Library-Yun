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
readyToSend = True
lastRx = 0


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

        # encode UDP packet into FramedBridge frame
        encodedFrame = fb.encode(bytearray(data))

        # if we haven't seen data from the DS in the last second
        # assume that the robot and DS were 'disconnected'
        # therefore, we can clear our outgoing buffer
        if (time.time() - lastRx) > 1.0:
            pendingTxBuffer = bytearray()
            readyToSend = True

        # append the bytes that just came in to our byte buffer
        pendingTxBuffer = pendingTxBuffer + encodedFrame

        # remember the last time we received data from the DS
        lastRx = time.time()


    # check if atmega is ready to receive and that we have data in our buffer
    if readyToSend and len(pendingTxBuffer) > 0:
        # write up to 64 bytes of our buffer to the atmega
        # if anything remains, store it back into our tx buffer
        sp.write(''.join(chr(b) for b in pendingTxBuffer[0:64]))
        pendingTxBuffer = pendingTxBuffer[64:]

        # need to wait for the atmega to clear us to transmit again
        readyToSend = False


    # read from serial
    while (sp.inWaiting() > 0):
        # read a single character (we don't want to block)
        dataIn = sp.read()

        # decode FramedBridge frame into robotopen packet
        fb.decode(bytearray(dataIn))


    # don't kill the CPU
    time.sleep(0.001)
