
#include "RobotOpen.h"

/* Constructor */
ROSolenoid::ROSolenoid(uint8_t channel)
{
    _channel = channel < 8 ? channel : 7;
}

void ROSolenoid::on() {
	RobotOpen.writeSolenoid(_channel, 0x01);
}

void ROSolenoid::off() {
	RobotOpen.writeSolenoid(_channel, 0x00);
}