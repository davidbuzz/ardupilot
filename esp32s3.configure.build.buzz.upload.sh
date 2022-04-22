#!/bin/bash
source ./modules/esp_idf/export.sh
export IDF_CCACHE_ENABLE=1
./waf configure --board=esp32s3buzz --debug
ESPBAUD=921600 ./waf plane --upload
#./waf plane --upload