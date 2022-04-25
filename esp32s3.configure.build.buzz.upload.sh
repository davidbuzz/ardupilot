#!/bin/bash
export IDF_CCACHE_ENABLE=1
source ./modules/esp_idf/export.sh

#WAFCACHE:
# https://gitlab.com/ita1024/waf/-/blob/master/waflib/extras/wafcache.py
# mkdir /tmp/wafcache > /dev/null
# export WAFCACHE=/tmp/wafcache
# export WAFCACHE_VERBOSITY=1
# export WAFCACHE_STATS=1
# export WAFCACHE_EVICT_INTERVAL_MINUTES=90
# ./waf configure --board=esp32s3devkit --debug --wafcache
# ESPBAUD=921600 ./waf plane 


#ccache:
#rm -rf /tmp/wafcache > /dev/null
unset WAFCACHE
unset CXX
unset CC
export CXX='ccache xtensa-esp32s3-elf-gcc' 
export CC='ccache xtensa-esp32s3-elf-g++' 
#export CXX='xtensa-esp32s3-elf-gcc' 
#export CC='xtensa-esp32s3-elf-g++' 
./waf configure --board=esp32s3devkit --debug
sleep 5
ESPBAUD=921600 ./waf plane -v --upload
#ESPBAUD=921600 ./waf plane 
