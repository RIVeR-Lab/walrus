#include "PWMChannel.h"

PWMChannel::PWMChannel()
{
  for (int l = 0; l < 100; l++)
    servos[l] = NULL;
}

bool PWMChannel::processCommand(byte command, byte* data, byte length, byte* response, byte* response_length)
{
  if (command == setPWM_cmd)
  {
    if (servos[data[0]] == NULL)
    {
      servos[data[0]] = new Servo();
      servos[data[0]]->attach(data[0]);
    }
    int val = (data[2] << 8) | data[1];
    servos[data[0]]->writeMicroseconds(val);
    (*response_length) = 0;
    return true;
  }
  else
    return false;
}
