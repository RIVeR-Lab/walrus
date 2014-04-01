#include <stdlib.h>

class ServoPin{
 private:
  const char* key;
  bool disabled;
  unsigned long period;
  unsigned long value;

 public:
  ServoPin(const char* key);
  ~ServoPin();

  int enable();
  int disable();

  int write(int ms);

};
