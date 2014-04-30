#include "Ping.h"

bool Ping::processCommand(byte command, byte* data, byte length, byte* response, byte* response_length)
{
  if (command == 0x01)
  {
    (*response_length) = 0;
    return true;
  }
  else
    return false;
};
