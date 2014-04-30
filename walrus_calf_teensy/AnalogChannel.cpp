#include "AnalogChannel.h"

bool AnalogChannel::processCommand(byte command, byte* data, byte lenght, byte* response, byte* response_length)
{
  if (command == getAnalog_cmd)
  {
    int val = analogRead(data[0]);
    response[2] = val;
    response[3] = val >> 8;
    (*response_length) = 2;
    return true;
  }
  else
    return false;
}
