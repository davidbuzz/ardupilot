#!/bin/bash
source ./modules/esp_idf/export.sh
xtensa-esp32s3-elf-gdb -x gdbinit.esp32s3 build/esp32s3buzz/esp-idf_build_s3/ardupilot.elf
