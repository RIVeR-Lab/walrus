extern "C"
{
  #include "event_gpio.h"
  #include "c_pwm.h"
  #include "c_adc.h"
}
#include <unistd.h>
#include <stdio.h>
#include "servo_pin.h"

int main(){
  ServoPin servoA("P8_13");
  usleep(200*1000);
  ServoPin servoB("P8_34");
  usleep(200*1000);
  ServoPin servoC("P8_45");
  usleep(200*1000);
  ServoPin servoD("P8_46");
  usleep(200*1000);
  ServoPin servoE("P9_14");
  usleep(200*1000);
  ServoPin servoF("P9_21");
  usleep(200*1000);
  ServoPin servoG("P9_22");
  usleep(200*1000);
  ServoPin servoH("P9_29");
  usleep(200*1000);
  ServoPin servoI("P9_42");

  printf("adc: %d\n", adc_setup());
  for(int i = 0; i<8; ++i){
    float val;
    read_value(i, &val);
    printf("\t adc: %f\n", val);
  }

  exports_cleanup();
  return 0;
}
