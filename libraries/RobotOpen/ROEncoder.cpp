
#include "RobotOpen.h"

/* Constructor */
ROEncoder::ROEncoder(uint8_t channel)
{
    _channel = channel < 8 ? channel : 7;
}

long ROEncoder::read() {
	return RobotOpen.readEncoder(_channel);
}

void ROEncoder::reset() {
	RobotOpen.resetEncoder(_channel);
}