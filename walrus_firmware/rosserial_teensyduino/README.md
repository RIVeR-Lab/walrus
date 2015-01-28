rosserial_teensyduino
=============

The teensyduino_sdk is the arduino 1.0.6 with the below modifications
 * Installed teensyduino 1.20 64bit (https://www.pjrc.com/teensy/td_download.html)
 * Installed Bounce2 library (http://playground.arduino.cc/Code/Bounce)


#### Usage
To use follow the rosserial_arduino instrucitons with the following modifications
Depend on rosserial_teensyduino
In the main CMakeLists.txt ```rosserial_configure_client``` with `rosserial_configure_teensyduino_client``` and remove the ```TOOLCHAIN_FILE``` argument
In the firmware CMakeLists.txt add ```include(${TEENSYDUINO})``` at the beginning of before calling the generate firmware commands
