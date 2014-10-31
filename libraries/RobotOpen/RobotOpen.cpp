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


// The interval for timed tasks to run
#define TIMED_TASK_INTERVAL_MS 100

// The interval for publishing DS data
#define DS_INTERVAL_MS 100

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

// Hold DS data
static boolean _dashboardPacketQueued = false;
static char _outgoingPacket[384];      // Data to publish to DS is stored into this array
static unsigned int _outgoingPacketSize = 1;

// Robot specific stuff
static boolean _enabled = false;            // Tells us if the robot is enabled or disabled
static uint8_t _controller_state = 1;       // 1 - NC, 2 - Disabled, 3 - Enabled (sent over SPI to coprocessor)
static unsigned long _lastPacket = 0;       // Keeps track of the last time (ms) we received data
static unsigned long _lastTimedLoop = 0;    // Keeps track of the last time the timed loop ran
static unsigned long _lastDSLoop = 0;       // Keeps track of the last time we published DS data

// milliseconds without receiving DS packet before we consider ourselves 'disconnected'
static int connection_timeout = 200;

static ROParameter* params[100];
static unsigned char paramsLength = 0;

/* SENT VIA SPI TO COPROCESSOR */
static uint8_t _pwmStates[12];
static uint8_t _solenoidStates[8];

static boolean acceptingDebugData = false;




// Networking support
static unsigned char _packetBuffer[384];
static unsigned int _packetBufferSize = 0;

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
    SPI.setClockDivider(SPI_CLOCK_DIV8);

    // setup DS packet
    _outgoingPacket[0] = 'd';

    delay(250); // Give Ethernet time to get ready

    // zero out solenoids and PWMs
    onDisable();

    // watchdog go!
    wdt_enable(WDTO_250MS);
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
    // update controller state w/ coprocessor

    // enable Slave Select
    digitalWrite(9, LOW);

    // coprocessor activate
    SPI.transfer(0xFF);

    // set controller state OPCODE
    SPI.transfer(0x01);
  
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

    // disable Slave Select
    digitalWrite(9, HIGH);
}

void RobotOpenClass::syncDS() {
    // feed watchdog
    wdt_reset();
  
    // detect disconnect
    if ((millis() - _lastPacket) > connection_timeout) {  // Disable the robot, drop the connection
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

    // Process any data sitting in the buffer
    FramedBridge.process();

    // Run user provided loops
    if (whileTimedTasks)
        whileTimedTasks();
    if (_enabled && whileEnabled)
        whileEnabled();
    if (!_enabled && whileDisabled)
        whileDisabled();

    // send update to coprocessor
    xmitCoprocessor();

    // run timed tasks
    if ((millis() - _lastTimedLoop) > TIMED_TASK_INTERVAL_MS) {
        acceptingDebugData = true;
        _lastTimedLoop = millis();
    } else {
        acceptingDebugData = false;
    }

    // ensure we only accept values for the DS packet for one debug loop and that data was actually published
    if (_outgoingPacketSize > 1)
        _dashboardPacketQueued = true;

    // publish DS data
    if ((millis() - _lastDSLoop) > DS_INTERVAL_MS) { 
        if (_outgoingPacketSize == 1 || _dashboardPacketQueued)
            publishDS();
        _lastDSLoop = millis();
    }
}

void RobotOpenClass::log(String data) {
    if (acceptingDebugData) {
        int dataLength = data.length();
        char logData[dataLength+1];

        logData[0] = 'p';

        for (int i=0; i < dataLength; i++) {
            logData[i+1] = data[i];
        }

        // xmit
        for (uint16_t i=0; i<dataLength+1; i++) {
            FramedBridge.write(logData[i]);
            FramedBridge.send();
        }
    }
}

unsigned int RobotOpenClass::calc_crc16(unsigned char *buf, unsigned short len) {
    unsigned short crc = 0;
    unsigned short i;
    for (i=0; i<len; i++)
        crc = ((crc >> 8) & 0xff) ^ pgm_read_word_near(crctab + (unsigned int)((crc ^ *buf++) & 0xff));
    return (crc);
}

boolean RobotOpenClass::publish(String id, unsigned char val) {
    if (_outgoingPacketSize+3+id.length() <= 384 && !_dashboardPacketQueued) {
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
    if (_outgoingPacketSize+4+id.length() <= 384 && !_dashboardPacketQueued) {
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
    if (_outgoingPacketSize+6+id.length() <= 384 && !_dashboardPacketQueued) {
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
    union u_tag {
        byte b[4];
        float fval;
    } u;
    u.fval = val;

    if (_outgoingPacketSize+6+id.length() <= 384 && !_dashboardPacketQueued) {
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
    _packetBufferSize = length;

    for(uint16_t i = 0; i < length; i++) {
        _packetBuffer[i] = payload[i];
    }

    parsePacket();  // Data is all set, time to parse through it
}

void RobotOpenClass::parsePacket() {
    // calculate crc16
    unsigned int crc16_recv = (_packetBuffer[_packetBufferSize - 2] << 8) | _packetBuffer[_packetBufferSize - 1];
    
    if (calc_crc16(_packetBuffer, _packetBufferSize - 2) == crc16_recv) {
        // control packet is 'c' + joystick data + crc16
        int frameLength = (_packetBufferSize - 3);
        int numParameters;
        int paramCount = 0;

        // VALID PACKET
        switch (_packetBuffer[0]) {
            case 'h': // heartbeat
              _enabled = false;
              _lastPacket = millis();
              break;

            case 'c': // control packet
              _enabled = true;
              _lastPacket = millis();
              _total_joysticks = (int)(frameLength/24);
              int i;

              for (i = 0; i < frameLength; i++) {
                if (i >= 0 && i < 24) {
                    // 1st joystick
                    _joy1[i] = _packetBuffer[i+1];
                }
                else if (i >= 24 && i < 48) {
                    // 2nd joystick
                    _joy2[i-24] = _packetBuffer[i+1];
                }
                else if (i >= 48 && i < 72) {
                    // 3rd joystick
                    _joy3[i-48] = _packetBuffer[i+1];
                }
                else if (i >= 72 && i < 96) {
                    // 4th joystick
                    _joy4[i-72] = _packetBuffer[i+1];
                }
                else {
                    break;
                }      
              }
              break;

            case 's': // set parameter packet
              numParameters = frameLength / 5;
              for (paramCount = 0; paramCount < numParameters; paramCount++) {
                writeParameter((uint8_t)_packetBuffer[(paramCount*5)+1], ((paramCount*5)+2));
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
        FramedBridge.send();
    }

    _outgoingPacketSize = 1;
    _dashboardPacketQueued = false;
}

void RobotOpenClass::writePWM(byte channel, uint8_t pwmVal) {
    if (channel < 12) {
        _pwmStates[channel] = pwmVal;
    }
}

long RobotOpenClass::readEncoder(byte channel) {
    // enable Slave Select
    digitalWrite(9, LOW);

    // coprocessor activate
    SPI.transfer(0xFF);

    // read encoder OPCODE
    SPI.transfer(0x02);

    // send encoder channel
    SPI.transfer(channel);

    // coprocessor buffer byte
    SPI.transfer(0x04);

    // grab encoder count off SPI bus
    long encoderCount = (SPI.transfer(0x04) << 24) | (SPI.transfer(0x04) << 16) | (SPI.transfer(0x04) << 8) | (SPI.transfer(0x04) & 0xFF);

    // disable Slave Select
    digitalWrite(9, HIGH);

    return encoderCount;
}

void RobotOpenClass::resetEncoder(byte channel) {
    // enable Slave Select
    digitalWrite(9, LOW);

    // coprocessor activate
    SPI.transfer(0xFF);

    // reset encoder OPCODE
    SPI.transfer(0x03);

    // send encoder channel
    SPI.transfer(channel);

    // disable Slave Select
    digitalWrite(9, HIGH);
}

void RobotOpenClass::writeSolenoid(byte channel, uint8_t state) {
    if (channel < 8) {
        _solenoidStates[channel] = state;
    }
}

void RobotOpenClass::addParameter(ROParameter* param) {
    params[paramsLength++] = param;
}

void RobotOpenClass::writeParameter(uint8_t location, unsigned int firstByte) {
    if (!_enabled) {
        EEPROM.write((location * 4), _packetBuffer[firstByte]);
        EEPROM.write((location * 4) + 1, _packetBuffer[firstByte+1]);
        EEPROM.write((location * 4) + 2, _packetBuffer[firstByte+2]);
        EEPROM.write((location * 4) + 3, _packetBuffer[firstByte+3]);
    }
}

void RobotOpenClass::sendParameters() {
    _outgoingPacket[0] = 'r';
    _outgoingPacketSize = 1;

    for (int i = 0; i < paramsLength; i++) {
        ROParameter prm = *params[i];

        if (_outgoingPacketSize+7+prm.label.length() <= 384) {
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
        FramedBridge.send();
    }

    // reset the outgoing packet vars
    _outgoingPacketSize = 1;
    _dashboardPacketQueued = false;
    _outgoingPacket[0] = 'd';
}

boolean RobotOpenClass::enabled() {
    return _enabled;
}

int RobotOpenClass::numJoysticks() {
    return _total_joysticks;
}