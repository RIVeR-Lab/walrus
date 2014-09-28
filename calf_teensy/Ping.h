#ifndef PING_H
#define PING_H
#include "CommandReceiver.h"

class Ping : public CommandReceiver
{
  public:
    virtual bool processCommand(byte command, byte* data, byte length, byte* response, byte* response_length);
};

#endif
