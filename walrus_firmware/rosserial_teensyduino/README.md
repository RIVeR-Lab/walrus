rosserial_teensyduino
=============

The teensyduino_sdk is the arduino 1.0.6 with the below modifications
 * Installed teensyduino 1.20 64bit (https://www.pjrc.com/teensy/td_download.html)
 * Installed Bounce2 library (http://playground.arduino.cc/Code/Bounce)
 * Installed MPL3115A2_Pressure library (https://github.com/sparkfun/MPL3115A2_Breakout)
 * Installed OneWire library (http://playground.arduino.cc/Learning/OneWirehttp://playground.arduino.cc/Learning/OneWire)
 * Installed custom ExternalADC library
 * Installed custom Bridge library
 * Installed custom TempHumid library
 * Installed custom Maxon library 
 * Modified file teensyduino_sdk\arduino-1.0.6\hardware\teensy\cores\teensy.core_pins.h to map arduous pin 46 to Port E Pin 3
 * Modified Servo library to only use Timer1 (Timer3 is required for analogWrite)


#### Usage
To use follow the rosserial_arduino instrucitons with the following modifications
Depend on rosserial_teensyduino
In the main CMakeLists.txt ```rosserial_configure_client``` with `rosserial_configure_teensyduino_client``` and remove the ```TOOLCHAIN_FILE``` argument
In the firmware CMakeLists.txt add ```include(${TEENSYDUINO})``` at the beginning of before calling the generate firmware commands
