#!/usr/bin/env bash

#Upload bootloader with chip erase
avrdude -c AVRISP2 -p AT90USB1286 -P $1 -U flash:w:`catkin_find --first-only --share walrus_bootloader BootloaderCDC.hex`:a

#Set Fuse bytes
avrdude -c AVRISP2 -p AT90USB1286 -P $1 -U efuse:w:0xF3:m
avrdude -c AVRISP2 -p AT90USB1286 -P $1 -U hfuse:w:0xD9:m
avrdude -c AVRISP2 -p AT90USB1286 -P $1 -U lfuse:w:0xDE:m

#Set lock bytes
avrdude -c AVRISP2 -p AT90USB1286 -P $1 -U lock:w:0xEF:m
