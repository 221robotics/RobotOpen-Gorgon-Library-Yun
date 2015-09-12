#ifndef ROENCODER_h
#define ROENCODER_h

#include "RobotOpenGY.h"


class ROEncoder
{
  public:
    ROEncoder(uint8_t);

    long read();
    float readCPS();
    void setSensitivity(uint16_t sensitivity);
    void setCPSSamplesToAverage(uint8_t samples);
    void reset();
    
  private:
    uint8_t _channel;
};



#endif