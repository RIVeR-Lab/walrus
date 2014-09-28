#include "calf_teensy_interface.h"
#include <unistd.h>

int main( int argc, char** argv ){
  CalfTeensyInterface teensy(100);
  teensy.open();
  printf("%d\n", teensy.ping());
    printf("%d\n", teensy.set_pwm(15, 1500));
    printf("%d\n", teensy.set_pwm(14, 1500));
    printf("%d\n", teensy.set_pwm(16, 1500));
    printf("%d\n", teensy.set_pwm(24, 1500));
    printf("%d\n", teensy.set_pwm(25, 1500));
    printf("%d\n", teensy.set_pwm(26, 1500));
  while(1){
    //printf("%d\n", teensy.ping());
    printf("%d, ", teensy.get_analog(42));
    printf("%d, ", teensy.get_analog(43));
    printf("%d, ", teensy.get_analog(44));
    printf("%d\n", teensy.get_analog(45));
    usleep(1000000);
  }
}
