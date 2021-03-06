
register_hardware_platform(${ARDUINO_SDK_PATH}/hardware/teensy)

add_definitions(-DLAYOUT_US_ENGLISH)
add_definitions(-DUSB_SERIAL)
add_definitions(-DUSBCON)
add_definitions(-DUSE_USBCON)
add_definitions(-DSerial_=usb_serial_class)
add_definitions(-D__AVR_AT90USB1286__)
set(teensypp2.build.f_cpu 16000000L)

