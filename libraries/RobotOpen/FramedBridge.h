/*
  FramedBridge.h - Frames over Yun's Bridge
  Created by Eric Barch, April 6, 2014.
*/

#ifndef FramedBridge_h
#define FramedBridge_h

#include "Arduino.h"

extern PROGMEM const short crctab[];


#define MAX_INCOMING_FRAME_SIZE 128
#define MAX_OUTGOING_FRAME_SIZE 512


typedef void FrameCallback(byte *payload, uint16_t length);


class FramedBridgeClass {
public:
    static void begin(long baud, FrameCallback *frameCallback);

    static void process();

    static void write(byte c);
    static void send();
    static void clear();
private:
    // Parse out packet
    static void parseFrame();
    
    // CRC16 checksum function
    static unsigned int calc_crc16(unsigned char *buf, unsigned short len);
};

extern FramedBridgeClass FramedBridge;

#endif