#!/usr/bin/env bash

#Upload bootloader with chip erase
avrdude -c AVRISP2 -p AT90USB1286 -P /dev/ttyACM0 -U flash:w:BootloaderCDC.hex:a

#Set Fuse bytes
avrdude -c AVRISP2 -p AT90USB1286 -P /dev/ttyACM0 -U efuse:w:0x9d:m
avrdude -c AVRISP2 -p AT90USB1286 -P /dev/ttyACM0 -U hfuse:w:0x9d:m
avrdude -c AVRISP2 -p AT90USB1286 -P /dev/ttyACM0 -U lfuse:w:0x9d:m

#Set lock bytes
avrdude -c AVRISP2 -p AT90USB1286 -P /dev/ttyACM0 -U lock:w:0x00:m
