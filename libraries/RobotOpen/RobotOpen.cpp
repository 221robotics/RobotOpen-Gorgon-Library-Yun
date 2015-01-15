/*
  RobotOpen.cpp - Library implementation of the RobotOpen Hardware found at www.RobotOpen.biz
  Created by Eric Barch, September 27, 2012.
*/

#include "Arduino.h"
#include <SPI.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include "RobotOpen.h"


// The interval for how often debug strings are transmitted
#define DEBUG_DATA_INTERVAL_MS 100

// The interval for publishing DS data
#define DS_PUBLISH_INTERVAL_MS 100

// Max packet sizes
#define INCOMING_PACKET_BUFFER_SIZE 128 // 128 allows for 25 parameters and 4 joysticks to be sent
#define OUTGOING_PACKET_BUFFER_SIZE 384
#define MAX_PARAMETERS              25  // 1 header byte + (5 * 25) + 2 crc bytes = 128 bytes max

// joystick data
static int _total_joysticks = 0;
static char _joy1[24];
static char _joy2[24];
static char _joy3[24];
static char _joy4[24];

// Pointers to loop callbacks
static LoopCallback *whileEnabled;
static LoopCallback *whileDisabled;
static LoopCallback *whileTimedTasks;

// hold onto references to parameter objects
static ROParameter* params[MAX_PARAMETERS];
static unsigned char paramsLength = 0;

// Hold DS/param data
static boolean _acceptingDebugData = false;
static boolean _acceptingDSData = false;
static char _outgoingPacket[OUTGOING_PACKET_BUFFER_SIZE];      // Data to publish to DS is stored into this array
static unsigned int _outgoingPacketSize = 0;

// Robot specific stuff
static boolean _enabled = false;            // Tells us if the robot is enabled or disabled
static uint8_t _controller_state = 1;       // 1 - NC, 2 - Disabled, 3 - Enabled (sent over SPI to coprocessor)
static unsigned long _lastControlPacket = 0;       // Keeps track of the last time (ms) we received data
static unsigned long _lastDebugDataPublish = 0;    // Keeps track of the last time the timed loop ran
static unsigned long _lastDSPublish = 0;       // Keeps track of the last time we published DS data

// milliseconds without receiving DS packet before we consider ourselves 'disconnected'
static int connection_timeout = 200;

// sent via SPI to coprocessor
static uint8_t _pwmStates[12];
static uint8_t _solenoidStates[8];

// Networking support
static unsigned char _incomingPacket[INCOMING_PACKET_BUFFER_SIZE];
static unsigned int _incomingPacketSize = 0;

// Class constructor
RobotOpenClass RobotOpen;


void RobotOpenClass::setTimeout(int new_timeout) {
    connection_timeout = new_timeout;
}

void RobotOpenClass::begin(LoopCallback *enabledCallback, LoopCallback *disabledCallback, LoopCallback *timedtasksCallback) {
    // Setup callbacks
    whileEnabled = enabledCallback;
    whileDisabled = disabledCallback;
    whileTimedTasks = timedtasksCallback;

    // we do NOT want to talk to the coprocessor yet
    pinMode(9, OUTPUT);
    digitalWrite(9, HIGH);

    // start FramedBridge
    FramedBridge.begin(115200, onData);

    // for use w/ stm32
    SPI.begin();

    // Give Ethernet time to get ready
    delay(250);

    // zero out solenoids and PWMs
    onDisable();

    // watchdog go!
    wdt_enable(WDTO_250MS);
}

void RobotOpenClass::beginCoprocessor() {
    // for use w/ stm32
    SPI.setClockDivider(SPI_CLOCK_DIV16);

    // enable Slave Select
    digitalWrite(9, LOW);

    // coprocessor activate
    SPI.transfer(0xFF);
    SPI.transfer(0x7F);
}

void RobotOpenClass::endCoprocessor() {
    // disable Slave Select
    digitalWrite(9, HIGH);
}

// This gets called once when the robot becomes disconnected or disabled
void RobotOpenClass::onDisable() {
    // neutral out all PWMs
    for (int i = 0; i < 12; i++) {
        _pwmStates[i] = 127;
    }
    // disable all Solenoids
    for (int i = 0; i < 8; i++) {
        _solenoidStates[i] = 0;
    }
    xmitCoprocessor();
}

void RobotOpenClass::xmitCoprocessor() {
    // this function sends PWM, solenoid, and enable state to the coprocessor

    // begin coprocessor transaction
    beginCoprocessor();

    // set controller state OPCODE
    SPI.transfer(COPROCESSOR_OP_SET_CONTROLLER_STATE);
  
    // write PWMs
    for (uint8_t i=0; i<12; i++) {
        SPI.transfer(_pwmStates[i]);
    }

    // write solenoids
    for (uint8_t i=0; i<8; i++) {
        SPI.transfer(_solenoidStates[i]);
    }

    // write LED state
    SPI.transfer(_controller_state);

    // end coprocessor transaction
    endCoprocessor();
}

void RobotOpenClass::attachDetachPWM(byte pwmChannel, bool attach) {
    // begin coprocessor transaction
    beginCoprocessor();

    // attach/detach OPCODE
    if (attach)
        SPI.transfer(COPROCESSOR_OP_ATTACH_PWM);
    else
        SPI.transfer(COPROCESSOR_OP_DETACH_PWM);

    // write PWM chan
    SPI.transfer(pwmChannel);

    // end coprocessor transaction
    endCoprocessor();
}

void RobotOpenClass::detachPWM(byte pwmChannel) {
    attachDetachPWM(pwmChannel, false);
}

void RobotOpenClass::attachPWM(byte pwmChannel) {
    attachDetachPWM(pwmChannel, true);
}

void RobotOpenClass::syncDS() {
    // feed watchdog
    wdt_reset();
  
    // detect disconnect
    if ((millis() - _lastControlPacket) > connection_timeout) {  // Disable the robot, drop the connection
        _enabled = false;
        _controller_state = 1;
        // NO CONNECTION
        onDisable();
    }
    else if (_enabled == true) {
        // ENABLED
        _controller_state = 3;
    }
    else {
        _controller_state = 2;
        // DISABLED
        onDisable();
    }

    // check for data from the DS
    FramedBridge.process();

    // send update to coprocessor
    xmitCoprocessor();

    // allow debug data to be published if the interval has expired
    if ((millis() - _lastDebugDataPublish) > DEBUG_DATA_INTERVAL_MS) {
        _acceptingDebugData = true;
        _lastDebugDataPublish = millis();
    }

    // allow DS data to be published this loop if the interval has expired
    if ((millis() - _lastDSPublish) > DS_PUBLISH_INTERVAL_MS) {
        // add dashboard header byte
        _outgoingPacket[0] = 'd';
        _outgoingPacketSize = 1;

        _acceptingDSData = true;
        _lastDSPublish = millis();
    }

    // Run user loops
    if (whileTimedTasks)
        whileTimedTasks();
    if (_enabled && whileEnabled)
        whileEnabled();
    else if (!_enabled && whileDisabled)
        whileDisabled();

    // make sure we accept no more debug data until the next interval
    _acceptingDebugData = false;

    // make sure we accept no more DS data until the next interval
    _acceptingDSData = false;

    // there is outgoing data to be sent, publish to DS
    if (_outgoingPacketSize > 1) {
        publishDS();
    } else {
        // header byte can be ignored, we had nothing to publish
        _outgoingPacketSize = 0;
    }
}

void RobotOpenClass::log(String data) {
    if (_acceptingDebugData) {
        int dataLength = data.length();
        char logData[dataLength+1];

        logData[0] = 'p';

        for (int i=0; i < dataLength; i++) {
            logData[i+1] = data[i];
        }

        // xmit
        for (uint16_t i=0; i<dataLength+1; i++) {
            FramedBridge.write(logData[i]);
        }
        FramedBridge.send();
    }
}

unsigned int RobotOpenClass::calc_crc16(unsigned char *buf, unsigned short len) {
    unsigned short crc = 0;
    unsigned short i;
    for (i=0; i<len; i++)
        crc = ((crc >> 8) & 0xff) ^ pgm_read_word_near(crctab + (unsigned int)((crc ^ *buf++) & 0xff));
    return (crc);
}

boolean RobotOpenClass::publish(String id, byte val) {
    if (_outgoingPacketSize+3+id.length() <= OUTGOING_PACKET_BUFFER_SIZE && _acceptingDSData) {
        _outgoingPacket[_outgoingPacketSize++] = 0xFF & (3+id.length());  // length
        _outgoingPacket[_outgoingPacketSize++] = 'c'; // type
        _outgoingPacket[_outgoingPacketSize++] = 0xFF & val;  // value
        for (int i = 0; i < id.length(); i++) {
            _outgoingPacket[_outgoingPacketSize++] = id[i];   // identifier
        }
        return true;
    }

    return false;
}

boolean RobotOpenClass::publish(String id, int val) {
    if (_outgoingPacketSize+4+id.length() <= OUTGOING_PACKET_BUFFER_SIZE && _acceptingDSData) {
        _outgoingPacket[_outgoingPacketSize++] = 0xFF & (4+id.length());  // length
        _outgoingPacket[_outgoingPacketSize++] = 'i'; // type
        _outgoingPacket[_outgoingPacketSize++] = (val >> 8) & 0xFF;  // value
        _outgoingPacket[_outgoingPacketSize++] = val & 0xFF;  // value
        for (int i = 0; i < id.length(); i++) {
            _outgoingPacket[_outgoingPacketSize++] = id[i];   // identifier
        }
        return true;
    }

    return false;
}

boolean RobotOpenClass::publish(String id, long val) {
    if (_outgoingPacketSize+6+id.length() <= OUTGOING_PACKET_BUFFER_SIZE && _acceptingDSData) {
        _outgoingPacket[_outgoingPacketSize++] = 0xFF & (6+id.length());  // length
        _outgoingPacket[_outgoingPacketSize++] = 'l'; // type
        _outgoingPacket[_outgoingPacketSize++] = (val >> 24) & 0xFF;  // value
        _outgoingPacket[_outgoingPacketSize++] = (val >> 16) & 0xFF;  // value
        _outgoingPacket[_outgoingPacketSize++] = (val >> 8) & 0xFF;  // value
        _outgoingPacket[_outgoingPacketSize++] = val & 0xFF;  // value
        for (int i = 0; i < id.length(); i++) {
            _outgoingPacket[_outgoingPacketSize++] = id[i];   // identifier
        }
        return true;
    }

    return false;
}

boolean RobotOpenClass::publish(String id, float val) {
    if (_outgoingPacketSize+6+id.length() <= OUTGOING_PACKET_BUFFER_SIZE && _acceptingDSData) {
        union u_tag {
            byte b[4];
            float fval;
        } u;
        u.fval = val;

        _outgoingPacket[_outgoingPacketSize++] = 0xFF & (6+id.length());  // length
        _outgoingPacket[_outgoingPacketSize++] = 'f'; // type
        _outgoingPacket[_outgoingPacketSize++] = u.b[3];  // value
        _outgoingPacket[_outgoingPacketSize++] = u.b[2];  // value
        _outgoingPacket[_outgoingPacketSize++] = u.b[1];  // value
        _outgoingPacket[_outgoingPacketSize++] = u.b[0];  // value
        for (int i = 0; i < id.length(); i++) {
            _outgoingPacket[_outgoingPacketSize++] = id[i];   // identifier
        }
        return true;
    }

    return false;
}

char* RobotOpenClass::getJoystick(char index) {
    if (index == 1 && _total_joysticks > 0)
        return _joy1;
    else if (index == 2 && _total_joysticks > 1)
        return _joy2;
    else if (index == 3 && _total_joysticks > 2)
        return _joy3;
    else if (index == 4 && _total_joysticks > 3)
        return _joy4;
    else
        return 0;
}

// called when a new frame comes in over FramedBridge
void RobotOpenClass::onData(byte *payload, uint16_t length) {
    _incomingPacketSize = length;

    for(uint16_t i = 0; i < length; i++) {
        _incomingPacket[i] = payload[i];
    }

    parsePacket();  // Data is all set, time to parse through it
}

void RobotOpenClass::parsePacket() {
    // calculate crc16
    unsigned int crc16_recv = (_incomingPacket[_incomingPacketSize - 2] << 8) | _incomingPacket[_incomingPacketSize - 1];
    
    if (calc_crc16(_incomingPacket, _incomingPacketSize - 2) == crc16_recv) {
        // control packet is 'c' + joystick data + crc16
        int frameLength = (_incomingPacketSize - 3);
        int numParameters;
        int paramCount = 0;

        // VALID PACKET
        switch (_incomingPacket[0]) {
            case 'h': // heartbeat
              _enabled = false;
              _lastControlPacket = millis();
              break;

            case 'c': // control packet
              _enabled = true;
              _lastControlPacket = millis();
              _total_joysticks = (int)(frameLength/24);
              int i;

              for (i = 0; i < frameLength; i++) {
                if (i >= 0 && i < 24) {
                    // 1st joystick
                    _joy1[i] = _incomingPacket[i+1];
                }
                else if (i >= 24 && i < 48) {
                    // 2nd joystick
                    _joy2[i-24] = _incomingPacket[i+1];
                }
                else if (i >= 48 && i < 72) {
                    // 3rd joystick
                    _joy3[i-48] = _incomingPacket[i+1];
                }
                else if (i >= 72 && i < 96) {
                    // 4th joystick
                    _joy4[i-72] = _incomingPacket[i+1];
                }
                else {
                    break;
                }      
              }
              break;

            case 's': // set parameter packet
              numParameters = frameLength / 5;
              for (paramCount = 0; paramCount < numParameters; paramCount++) {
                writeParameter((uint8_t)_incomingPacket[(paramCount*5)+1], ((paramCount*5)+2));
              }
              break;

            case 'g': // get parameters packet
              sendParameters();
              break;

            default:
              // ignore the packet
              break;
        }
    }
}

void RobotOpenClass::publishDS() {
    // xmit
    for (uint16_t i=0; i<_outgoingPacketSize; i++) {
        FramedBridge.write(_outgoingPacket[i]);
    }
    FramedBridge.send();

    _outgoingPacketSize = 0;
}

void RobotOpenClass::writePWM(byte channel, uint8_t pwmVal) {
    if (channel < 12) {
        _pwmStates[channel] = pwmVal;
    }
}

long RobotOpenClass::readEncoder(byte channel) {
    // begin coprocessor transaction
    beginCoprocessor();

    // read encoder OPCODE
    SPI.transfer(COPROCESSOR_OP_GET_ENCODER);

    // send encoder channel
    SPI.transfer(channel);

    // coprocessor buffer byte
    SPI.transfer(0x04);

    // grab encoder count off SPI bus
    long encoderCount = (SPI.transfer(0x04) << 24) | (SPI.transfer(0x04) << 16) | (SPI.transfer(0x04) << 8) | (SPI.transfer(0x04) & 0xFF);

    // end coprocessor transaction
    endCoprocessor();

    return encoderCount;
}

float RobotOpenClass::readEncoderCPS(byte channel) {
    // begin coprocessor transaction
    beginCoprocessor();

    // read encoder CPS OPCODE
    SPI.transfer(COPROCESSOR_OP_GET_ENCODER_CPS);

    // send encoder channel
    SPI.transfer(channel);

    // coprocessor buffer byte
    SPI.transfer(0x04);

    // grab encoder count off SPI bus
    union {
        float f;
        uint8_t b[4];
    } u;
    u.b[0] = SPI.transfer(0x04);
    u.b[1] = SPI.transfer(0x04);
    u.b[2] = SPI.transfer(0x04);
    u.b[3] = SPI.transfer(0x04);

    // end coprocessor transaction
    endCoprocessor();

    return u.f;
}

void RobotOpenClass::setEncoderSensitivity(byte channel, uint16_t sensitivity) {
    // begin coprocessor transaction
    beginCoprocessor();

    // set encoder sensitivty OPCODE
    SPI.transfer(COPROCESSOR_OP_SET_ENCODER_SENSITIVITY);

    // send encoder channel
    SPI.transfer(channel);

    // send sensitivty
    SPI.transfer((sensitivity << 8) & 0xFF);
    SPI.transfer(sensitivity & 0xFF);

    // end coprocessor transaction
    endCoprocessor();
}

void RobotOpenClass::setEncoderSamplesToAverage(byte channel, uint8_t samples) {
    // begin coprocessor transaction
    beginCoprocessor();

    // set encoder sensitivty OPCODE
    SPI.transfer(COPROCESSOR_OP_SET_ENCODER_AVERAGE);

    // send encoder channel
    SPI.transfer(channel);

    // send sensitivty
    SPI.transfer(samples);

    // end coprocessor transaction
    endCoprocessor();
}

void RobotOpenClass::resetEncoder(byte channel) {
    // begin coprocessor transaction
    beginCoprocessor();

    // reset encoder OPCODE
    SPI.transfer(COPROCESSOR_OP_RESET_ENCODER);

    // send encoder channel
    SPI.transfer(channel);

    // end coprocessor transaction
    endCoprocessor();
}

void RobotOpenClass::writeSolenoid(byte channel, uint8_t state) {
    if (channel < 8) {
        _solenoidStates[channel] = state;
    }
}

void RobotOpenClass::addParameter(ROParameter* param) {
    // add parameter if it fits
    if (paramsLength < MAX_PARAMETERS)
        params[paramsLength++] = param;
}

void RobotOpenClass::writeParameter(uint8_t location, unsigned int firstByte) {
    if (!_enabled) {
        EEPROM.write((location * 4), _incomingPacket[firstByte]);
        EEPROM.write((location * 4) + 1, _incomingPacket[firstByte+1]);
        EEPROM.write((location * 4) + 2, _incomingPacket[firstByte+2]);
        EEPROM.write((location * 4) + 3, _incomingPacket[firstByte+3]);
    }
}

void RobotOpenClass::sendParameters() {
    _outgoingPacket[0] = 'r';
    _outgoingPacketSize = 1;

    for (int i = 0; i < paramsLength; i++) {
        ROParameter prm = *params[i];

        if (_outgoingPacketSize+7+prm.label.length() <= OUTGOING_PACKET_BUFFER_SIZE) {
            _outgoingPacket[_outgoingPacketSize++] = 0xFF & (7+prm.label.length());         // length
            _outgoingPacket[_outgoingPacketSize++] = 0xFF & (prm.location);                 // address (0-99)
            _outgoingPacket[_outgoingPacketSize++] = prm.type;                              // type
            _outgoingPacket[_outgoingPacketSize++] = EEPROM.read(prm.location * 4);         // val1
            _outgoingPacket[_outgoingPacketSize++] = EEPROM.read((prm.location * 4) + 1);   // val2
            _outgoingPacket[_outgoingPacketSize++] = EEPROM.read((prm.location * 4) + 2);   // val3
            _outgoingPacket[_outgoingPacketSize++] = EEPROM.read((prm.location * 4) + 3);   // val4
            for (int j = 0; j < prm.label.length(); j++) {
                _outgoingPacket[_outgoingPacketSize++] = prm.label[j];                      // identifier
            }
        } else {
            break;
        }
    }

    // xmit
    for (uint16_t i=0; i<_outgoingPacketSize; i++) {
        FramedBridge.write(_outgoingPacket[i]);
    }
    FramedBridge.send();

    // no more outgoing data
    _outgoingPacketSize = 0;
}

boolean RobotOpenClass::enabled() {
    return _enabled;
}

int RobotOpenClass::numJoysticks() {
    return _total_joysticks;
}