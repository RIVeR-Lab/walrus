#include <Wire.h>
#include <Servo.h>
#include "AnalogChannel.h"
#include "PWMChannel.h"
#include "Ping.h"
#include "CommandReceiver.h"
#include "Comm.h"

Comm* comm;
AnalogChannel* analog;
Ping* ping;
PWMChannel* pwm;

void setup()
{
  comm = new Comm();
  
  analog = new AnalogChannel();
  pwm = new PWMChannel();
  ping = new Ping();
  
  comm->RegisterCommandReceiver(analog);
  comm->RegisterCommandReceiver(pwm);
  comm->RegisterCommandReceiver(ping);
}

void loop()
{
  comm->sustain();
}

