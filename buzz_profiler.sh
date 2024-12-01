#!/bin/bash

# https://poormansprofiler.org/

#source ./modules/esp_idf/export.sh

elf=build/esp32s3empty/esp-idf_build/ardupilot.elf
gdbdev=`ls /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*`
gdb=xtensa-esp32s3-elf-gdb
nsamples=10
sleeptime=0
#nthreads=14

# write to a .init file to be sourced by gdb
# this specificially does NOT reset the device or halt it, we want to keep the app running while probing.
touch gdb.profiler.init
rm -f ./gdb.profiler.init*

cat > ./gdb.profiler.init <<EOF
target remote :3333
set print pretty
set confirm off
set pagination 0
set print asm-demangle on
set remote hardware-watchpoint-limit 2
set print thread-events off
thread apply all bt
EOF

# unused
#mon reset halt
#flushregs
#thb app_main
#c
# thread apply all bt full
# thread apply all bt
# i threads

# a single call to demo it works on its own.
#$gdb $elf --nx --quiet --batch -x ./gdb.profiler.init
#$gdb $elf --nx --quiet --batch -x ./gdb.profiler.init 
#exit

>&2 echo -n "Sampling..." $nsamples " => "
for x in $(seq 1 $nsamples)
  do
    >&2 echo -n "$x "
    $gdb $elf --nx --quiet --batch -x ./gdb.profiler.init 2>&1 | grep -v warning
    sleep $sleeptime
  done | \
awk '
  BEGIN { s = ""; } 
  /^Thread/ { print s; s = ""; } 
  /^#/ { if (s != "" ) { s = s "," $4} else { s = $4 } } 
  END { print s }' | \
sort | uniq -c | sort -r -n -k 1,1