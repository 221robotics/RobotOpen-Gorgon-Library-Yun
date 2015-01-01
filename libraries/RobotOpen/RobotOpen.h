/*
  RobotOpen.h - Library implementation of the RobotOpen Hardware found at www.RobotOpen.biz
  Created by Eric Barch, September 27, 2012.
*/

#ifndef RobotOpen_h
#define RobotOpen_h

#include "Arduino.h"
#include <ROJoystick.h>
#include <ROPWM.h>
#include <ROAnalog.h>
#include <RODigitalIO.h>
#include <RODashboard.h>
#include <ROSolenoid.h>
#include <ROStatus.h>
#include <ROTimer.h>
#include <ROParameter.h>
#include <ROEncoder.h>
#include <FramedBridge.h>


// coprocessor opcodes
#define COPROCESSOR_OP_SET_CONTROLLER_STATE     0x01
#define COPROCESSOR_OP_GET_ENCODER              0x02
#define COPROCESSOR_OP_RESET_ENCODER            0x03
#define COPROCESSOR_OP_GET_ENCODER_CPS          0x04
#define COPROCESSOR_OP_SET_ENCODER_SENSITIVITY  0x05
#define COPROCESSOR_OP_ATTACH_PWM               0x06
#define COPROCESSOR_OP_DETACH_PWM               0x07
#define COPROCESSOR_OP_SET_ENCODER_AVERAGE      0x08
#define COPROCESSOR_OP_RESET                    0x64


typedef void LoopCallback();


class RobotOpenClass {
public:
    // Configure timeout until robot considers itself disconnected
    static void setTimeout(int new_timeout);

    // Fire up the RobotOpen object and get things running
    static void begin(LoopCallback *enabledCallback, LoopCallback *disabledCallback, LoopCallback *timedtasksCallback);
    
    // Exchange data with DS
    static void syncDS();

    // Log data to DS
    static void log(String data);
    
    // Tells us if the robot is enabled
    static boolean enabled();

    // How many joysticks are being received
    static int numJoysticks();

    // Calls for ROShield
    static void xmitCoprocessor();

    // Overloaded calls to publish back data
    static boolean publish(String id, byte val);
    static boolean publish(String id, int val);
    static boolean publish(String id, long val);
    static boolean publish(String id, float val);

    static char* getJoystick(char index);

    static void writePWM(byte channel, byte pwmVal);
    static void writeSolenoid(byte channel, uint8_t state);

    static int32_t readEncoder(byte channel);
    static float readEncoderCPS(byte channel);
    static void setEncoderSensitivity(byte channel, uint16_t sensitivity);
    static void setEncoderSamplesToAverage(byte channel, uint8_t samples);
    static void resetEncoder(byte channel);

    static void addParameter(ROParameter* param);

    static void detachPWM(byte pwmChannel);
    static void attachPWM(byte pwmChannel);

private:
    // Dumps data back to the DS
    static void publishDS();

    // This gets called once when the robot becomes disconnected or disabled
    static void onDisable();

    // Callback for FramedBridge
    static void onData(byte *payload, uint16_t length);

    // Parse out a DS packet
    static void parsePacket();

    // Attach or Detach a PWM pin
    static void attachDetachPWM(byte pwmChannel, bool attach);

    // Grab UDP data
    static void handleData();

    // Update parameter in EEPROM
    static void writeParameter(uint8_t location, unsigned int firstByte);

    // Send all parameters to DS
    static void sendParameters();
    
    // CRC16 checksum function
    static unsigned int calc_crc16(unsigned char *buf, unsigned short len);

    // Coprocessor methods
    static void beginCoprocessor();
    static void endCoprocessor();
};

extern RobotOpenClass RobotOpen;

#endif