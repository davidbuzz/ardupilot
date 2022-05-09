#!/bin/bash
source ./modules/esp_idf/export.sh
# this branch of esp32 puts the partition table at 0x10000 and ardupilot.bin at 0x20000
# eg:  esptool.py --chip esp32s3 -b 921600 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 8MB 0x0 bootloader/bootloader.bin 0x20000 ardupilot.bin 0x10000 partition_table/partition-table.bin
# if your openocd session says stuff like: 
#Error: Failed to get flash maps (4294967295)!
#Warn : Failed to get flash mappings (-4)!
# then it means it won't set/clear breakpoints in FLASH , as it won't know flash offsets.
# our 'openocd supports special command which can be used to set arbitrary location of application image to debug'
# https://docs.espressif.com/projects/esp-idf/en/v4.2-beta1/esp32/api-guides/jtag-debugging/tips-and-quirks.html#flash-mappings-vs-sw-flash-breakpoints
# ...write_flash..  ends in a sequence of <address> <filename> in that order
#openocd -f board/esp32s3-builtin.cfg -c "init; halt; esp32s3 appimage_offset 0x20000" -d2
openocd -f board/esp32s3-builtin.cfg -c "init ; halt ;esp32s3 appimage_offset 0x20000 ; resume" -d3


# segger tips;
# from video: https://www.youtube.com/watch?v=rLXjd4VxYaU&ab_channel=Kevin
# flash with sdk mods and code mods:
# run/connect openocd then:
# telnet localhost 4444
# esp sysview start file://pro.bin file://app.bin 1 -1 -1 0 0
# ...wait....
# esp sysview stop