/*
  FramedBridge.cpp - Frames over Yun's Bridge
  Created by Eric Barch, April 6, 2014.
*/

#include "Arduino.h"
#include "FramedBridge.h"


// Class constructor
FramedBridgeClass FramedBridge;

// rx vars
static byte _incomingPacket[MAX_INCOMING_FRAME_SIZE];
static uint16_t _incomingPacketIndex = 0;
static bool _escaped = false;

// tx vars
static byte _outgoingPacket[MAX_OUTGOING_FRAME_SIZE];
static uint16_t _outgoingPacketIndex = 0;

static unsigned long last_recv = 0;
static bool initial_recv = false;

// callback for data
static FrameCallback *onFrame;


void FramedBridgeClass::begin(long baud, FrameCallback *frameCallback) {
    onFrame = frameCallback;

    Serial1.begin(baud);
}

void FramedBridgeClass::validPacket() {
    initial_recv = true;
}

void FramedBridgeClass::process() {
    bool processedData = false;

    // data available from Yun processor?
    while (Serial1.available() > 0) {
        // read next byte
        byte c = Serial1.read();

        // next char will be escaped
        if (c == 0x7D) {
            // ESCAPE
            _escaped = true;
        } else if (c == 0x7E) {
            // FLAG
            parseFrame();

            _incomingPacketIndex = 0;
            _escaped = false;
        } else {
            // PAYLOAD

            // XOR escaped char w/ 0x20
            if (_escaped)
                c ^= 0x20;

            // we are no longer escaped (if we were)
            _escaped = false;

            // don't buffer any data past MAX_FRAME_SZE
            if (_incomingPacketIndex >= MAX_INCOMING_FRAME_SIZE)
                return;

            // buffer incoming byte
            _incomingPacket[_incomingPacketIndex++] = c;
        }

        processedData = true;
    }

    if (processedData && ((millis() - last_recv) <= 1000) && initial_recv) {
        // write our flow control byte of 0x7F to indicate that our buffer is now empty
        Serial1.write(0x7F);
    }
}


void FramedBridgeClass::parseFrame() {
    if (_incomingPacketIndex == 0)
        return;

    // keep track of the last time we parsed a packet
    last_recv = millis();

    // if the callback exists, fire it with the frame and frame length as parameters
    if (onFrame)
        onFrame(_incomingPacket, _incomingPacketIndex);
}


void FramedBridgeClass::write(byte c) {
    // we need to escape any 'frame' or 'escape' bytes
    if (c == 0x7D || c == 0x7E || c == 0x7F) {
        // make sure there are at least 2 free bytes (escape + char)
        if (_outgoingPacketIndex + 2 > MAX_OUTGOING_FRAME_SIZE)
            return;

        _outgoingPacket[_outgoingPacketIndex++] = c;
    } else {
        // make sure there is at least 1 free byte (char)
        if (_outgoingPacketIndex + 1 > MAX_OUTGOING_FRAME_SIZE)
            return;

        _outgoingPacket[_outgoingPacketIndex++] = c;
    }
}

void FramedBridgeClass::send() {
    // make sure we've received data recently, otherwise discard this packet
    if ((millis() - last_recv) > 1000 || !initial_recv) {
        clear();
        return;
    }

    // write buffer
    for (uint16_t i = 0; i < _outgoingPacketIndex; i++) {
        if (_outgoingPacket[i] == 0x7D || _outgoingPacket[i] == 0x7E || _outgoingPacket[i] == 0x7F) {
            Serial1.write(0x7D);
            Serial1.write(_outgoingPacket[i] ^ 0x20);
        } else {
            Serial1.write(_outgoingPacket[i]);
        }
    }

    // frame flag
    Serial1.write(0x7E);

    // reset packet index
    clear();
}

void FramedBridgeClass::clear() {
    _outgoingPacketIndex = 0;
}