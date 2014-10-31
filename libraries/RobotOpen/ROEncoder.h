#ifndef ROENCODER_h
#define ROENCODER_h

#include <RobotOpen.h>


class ROEncoder
{
  public:
    ROEncoder(uint8_t);

    long read();
    void reset();
    
  private:
    uint8_t _channel;
};



#endif