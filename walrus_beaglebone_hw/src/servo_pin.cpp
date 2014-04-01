extern "C"
{
  #include "c_pwm.h"
}
#include <unistd.h>
#include <stdio.h>
#include "servo_pin.h"

#define INIT_RETRY_DELAY (200)
#define NUM_INIT_RETRYS 10
#define NUM_INIT_RETRY_BEFORE_UNLOAD 3

ServoPin::ServoPin(const char* key_): key(key_), disabled(true),
				      period(1000), value(0){
  int init_result = 0;
  for(int i = 0; i<NUM_INIT_RETRYS; ++i){
    init_result = pwm_start(key, 20, 50, 0);
    if(init_result == 1){
      printf("Initialized PWM on %s\n", key);
      break;
    }
    printf("Failed to PWM on %s, trying again in %dms\n", key, INIT_RETRY_DELAY);
    if(i%NUM_INIT_RETRY_BEFORE_UNLOAD == NUM_INIT_RETRY_BEFORE_UNLOAD-1){
      printf("Disabling PWM on %s, before trying again\n", key);
      pwm_disable(key);
    }
    usleep(INIT_RETRY_DELAY*1000);//sleep 200ms and try again
  }
  if(init_result != 1)
    printf("Failed to PWM on %s, after %d tries\n", key, NUM_INIT_RETRYS);

}
ServoPin::~ServoPin(){
  disable();
}

int ServoPin::enable(){
  return 0;//pwm_start(key);
}

int ServoPin::disable(){
  return 0;//pwm_disable(key);
}
