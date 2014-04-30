#ifndef PWM_H
#define PWM_H

#include "CommandReceiver.h"
#include <Servo.h>


class PWMChannel : public CommandReceiver
{
  private:
    const static byte setPWM_cmd = 0x04;
    Servo* servos[100];
  
  public:
    PWMChannel();
    virtual bool processCommand(byte command, byte* data, byte length, byte* response, byte* response_length);
};

#endif
