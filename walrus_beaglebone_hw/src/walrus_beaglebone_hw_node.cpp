extern "C"
{
  #include "event_gpio.h"
}
#include <unistd.h>
#include <stdio.h>

int main(){
  printf("export %d\n", gpio_export(48));
    usleep(500000);
  printf("direction %d\n", gpio_set_direction(48, OUTPUT));
  for(int i = 0; i<10; ++i){
    usleep(500000);
    printf("high %d\n", gpio_set_value(48, HIGH));
    unsigned int val;
    //gpio_get_value(48, &val);
    printf("val %d\n", val);
    usleep(500000);
    printf("low %d\n", gpio_set_value(48, LOW));
    //gpio_get_value(48, &val);
    printf("val %d\n", val);
  }
  printf("direction %d\n", gpio_set_direction(48, INPUT));
  printf("unexport %d\n", gpio_unexport(50));

  exports_cleanup();
  return 0;
}
