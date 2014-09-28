#ifndef ANALOG_H
#define ANALOG_H
#include "CommandReceiver.h"

class AnalogChannel : public CommandReceiver
{
  private:
    static const byte getAnalog_cmd = 0x05;
    
  public:
    virtual bool processCommand(byte command, byte* data, byte length, byte* response, byte* response_length);
};

#endif
