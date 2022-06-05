#!/bin/bash

# esp32 cmake+scripting has a weird bug right now where it won't build with ccache enabled, so for now --disable-scripting is a work-around to allow faster ccache-based builds to work.

# on my computer, a typical complete esp32 build without ccache takes 20+ minutes, 
# and after a 'rm -rf build' and with a primed ccache, it takes 4 minutes, so if iterating on esp32 a lot, its worth
# disabling scripting and enabling ccache for turning bulds around faster.

#hwdef hacking:
#cd ardupilot/libraries/AP_HAL_ESP32/hwdef/scripts
#python esp32_hwdef.py ../esp32buzz/hwdef3.dat 
#cp /tmp/hwdef.h .

# 
rm -rf build

#export IDF_CCACHE_ENABLE=1
unset IDF_CCACHE_ENABLE
source ./modules/esp_idf/export.sh

#ccache:
unset CXX
unset CC
#export CXX='ccache xtensa-esp32s3-elf-gcc' 
#export CC='ccache xtensa-esp32s3-elf-g++' 
#export CXX='xtensa-esp32s3-elf-gcc' 
#export CC='xtensa-esp32s3-elf-g++' 
#./waf configure --board=esp32buzz --debug --toolchain=xtensa-esp32s3-elf --disable-scripting
./waf configure --board=esp32buzz --debug --disable-scripting
#./waf configure --board f103-GPS
echo "about to build in 3 sec..."
sleep 3

#with ccache, but without scripting, its ok
time ESPBAUD=921600 ./waf AP_Periph --jobs=4 -v -v

# -v
#--upload ?

