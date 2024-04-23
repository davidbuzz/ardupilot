#!/bin/bash -x

cd /home/buzz2/ardupilot

source ./cmake.last.env.bin

. ./modules/esp_idf/export.sh

cd $BUILDDIR


DEV=`ls /dev/tty[UA]*`
echo "flashing to device: DEV=$DEV"

# before we get here, we've already run the cmake.xx.configure.sh  which ran 'cmake ..' in this build directory as part of the 'configure' script
# here's a few ways to trigger the build with various verbosity levels and parallelism..
# make
# make VERBOSE=1
# make VERBOSE=1
# make VERBOSE=1 -j16
# cmake --build .
# make flash -j16  # this uses idf.py autodetection of the serial port
ESPPORT=`ls /dev/tty[UA]*` make flash -j16 # this is the most common way to build the project
cd ..