#!/usr/bin/env bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

cloc --exclude-dir="walrus_web_server/web,walrus_web_server/build,walrus_web_server/bower_components,walrus_web_server/node_modules,walrus_web_server/libs,walrus_firmware/rosserial_teensyduino/teensyduino_sdk,walrus_firmware/walrus_bootloader/src" $@ $DIR
