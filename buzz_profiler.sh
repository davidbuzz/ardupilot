#!/bin/bash

# https://poormansprofiler.org/
#source ./modules/esp_idf/export.sh

elf=build/esp32s3empty/esp-idf_build/ardupilot.elf
gdbdev=`ls /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*`
gdb=xtensa-esp32s3-elf-gdb
# if you want to sample the full boot sequence it takes a couple of minutes
#nsamples=120
nsamples=500
sleeptime=1
#nthreads=14

# write to a .init file to be sourced by gdb
# this specificially does NOT reset the device or halt it, we want to keep the app running while probing.
touch gdb.profiler.init
rm -f ./gdb.profiler.init

cat > ./gdb.profiler.init <<EOF
target remote :3333
set confirm off
#handle SIGINT nostop print nopass
handle SIGINT stop print nopass
catch signal SIGUSR1 
set print pretty
set pagination 0
set print asm-demangle on
set remote hardware-watchpoint-limit 2
set print thread-events off
thread apply all bt
c
EOF

>&2 echo -n "Sampling..." $nsamples " => "
# gdb in background so we can send it signals later.
#$gdb $elf --nx --quiet --batch -x ./gdb.profiler.init 
#2>&1 | grep -v warning &
sleep 5
for x in $(seq 1 $nsamples)
  do
    >&2 echo -n "$x "
    $gdb $elf --nx --quiet --batch -x ./gdb.profiler.init 2>&1 | grep -v warning &
    sleep $sleeptime
    # SIGINT stops it running
    #pkill --signal SIGINT -f xtensa-esp-elf-gdb
    # SIGUSR1 terminates GDB and leaves the app running
    pkill --signal SIGUSR1 -f xtensa-esp-elf-gdb
  done | \
awk '
  BEGIN { s = ""; } 
  /^Thread/ { print s; s = ""; } 
  /^#/ { if (s != "" ) { s = s "," $4} else { s = $4 } } 
  END { print s }' | \
sort | uniq -c | sort -r -n -k 1,1