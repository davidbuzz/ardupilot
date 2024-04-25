#!/bin/bash

cd /home/buzz2/ardupilot

source ./cmake.last.env.bin

echo "--------------------------------------------------------------------------------"
echo "building this target..."
cat ./cmake.last.env.bin | grep export
echo "--------------------------------------------------------------------------------"


. ./modules/esp_idf/export.sh 2>&1 > /dev/null

cd $BUILDDIR


# before we get here, we've already run the cmake.xx.configure.sh  which ran 'cmake ..' in this build directory as part of the 'configure' script
# here's a few ways to trigger the build with various verbosity levels and parallelism..
# make
# make VERBOSE=1
# make VERBOSE=1
# make VERBOSE=1 -j16
# cmake --build .
make -j12 # this is the most common way to build the project
cd ..