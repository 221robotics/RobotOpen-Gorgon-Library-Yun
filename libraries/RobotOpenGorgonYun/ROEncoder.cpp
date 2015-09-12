#include "RobotOpenGY.h"

/* Constructor */
ROEncoder::ROEncoder(uint8_t channel)
{
    _channel = channel < 8 ? channel : 7;
}

long ROEncoder::read() {
	return RobotOpen.readEncoder(_channel);
}

float ROEncoder::readCPS() {
	return RobotOpen.readEncoderCPS(_channel);
}

void ROEncoder::setSensitivity(uint16_t sensitivity) {
	// default is 4
	RobotOpen.setEncoderSensitivity(_channel, sensitivity);
}

void ROEncoder::setCPSSamplesToAverage(uint8_t samples) {
	// default is 9
	RobotOpen.setEncoderSamplesToAverage(_channel, samples);
}

void ROEncoder::reset() {
	RobotOpen.resetEncoder(_channel);
}