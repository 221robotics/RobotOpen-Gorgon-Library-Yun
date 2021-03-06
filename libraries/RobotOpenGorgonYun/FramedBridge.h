/*
  FramedBridge.h - Frames over Yun's Bridge
  Created by Eric Barch, April 6, 2014.
*/

#ifndef FramedBridge_h
#define FramedBridge_h

#include "Arduino.h"


#define MAX_INCOMING_FRAME_SIZE 150
#define MAX_OUTGOING_FRAME_SIZE 450


typedef void FrameCallback(byte *payload, uint16_t length);


class FramedBridgeClass {
public:
    static void begin(long baud, FrameCallback *frameCallback);

    static void process();

    static void write(byte c);
    static void send();
    static void clear();

    static void validPacket();
private:
    // Parse out packet
    static void parseFrame();
};

extern FramedBridgeClass FramedBridge;

#endif