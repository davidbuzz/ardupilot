#!/bin/bash
source ./modules/esp_idf/export.sh
openocd -f board/esp32s3-builtin.cfg -d2
