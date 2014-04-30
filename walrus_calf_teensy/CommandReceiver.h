#ifndef COMMAND_RECEIVER_H
#define COMMAND_RECEIVER_H

#include <Arduino.h>

class CommandReceiver
{
  public:
    virtual bool processCommand(byte command, byte* data, byte length, byte* response, byte* response_length) = 0;
};

#endif
