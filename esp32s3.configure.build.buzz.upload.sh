#!/bin/bash
source ./modules/esp_idf/export.sh
./waf configure --board=esp32s3buzz --debug
./waf plane --upload