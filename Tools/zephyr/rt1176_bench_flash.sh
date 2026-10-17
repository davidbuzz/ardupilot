#!/bin/bash
# Flash the verified rt1176 image. Needs the serial port FREE (MAVProxy closed).
cd /home/buzz/arduzeph
if fuser /dev/ttyACM0 >/dev/null 2>&1; then echo "PORT BUSY:"; fuser -v /dev/ttyACM0; exit 1; fi
# an orphaned uploader steals the bootloader window: match on the interpreter path
# match on the interpreter ($2), never on the pattern text: pgrep -f also matches
# the shell of whoever invoked this script if that command line mentions uploader.py
for p in $(pgrep -af 'uploader.py' | awk '$2 ~ /python/ {print $1}'); do echo "killing orphan uploader $p"; kill $p; done
# a GLOB, on purpose: the bootloader enumerates as ...-BL_...-if-bootloader, and
# uploader.py expands the glob itself, so it finds the board in either state.
PORT='/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176*'
echo "port: $PORT"
python3 Tools/scripts/uploader.py --port "$PORT" build/mr_vmu_rt1176/bin/mr_vmu_rt1176.apj
