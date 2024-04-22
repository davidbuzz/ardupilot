#!/bin/bash

find . -type f | grep -v SITL | grep -v AP_HAL_Linux | grep -v AP_HAL_ChibiOS | grep -v AP_IOMCU | grep -v AP_JSON  | grep -v AP_NavEKF/Models | grep -v AP_Networking | grep -v AP_DDS | grep -v AP_Scripting |grep -v '/examples/' | grep -v LICENSE | grep -v README | grep -v '/tests/' | grep -v '.m$' | grep -v '.py' | grep -v '.a$' |grep -v wscript | grep -v '.dat$'| grep -v '.dat$' | grep -v '.map$' | grep -v '.kt$' | grep -v '.TXT$' |grep -v '.txt$'| grep -v .git | grep -v '.csv' | grep -v '/targets/' | grep -v '/benchmarks/' | grep -v '/generated/' | grep -v '.sh$' | grep -v '/fonts/' | grep -v '/tools/' | grep -v '/Tools/' | grep -v '.xml$' | grep -v 'CMakeFiles' | grep -v '.cmake' | grep -v '/doc/' | grep -v 'GCS_Dummy' | grep -v 'IGNORE' | grep -v 'add_library' | grep -v './libs.short' | grep -v './t2' | sort -u > libs.short.readable.txt

echo 'add_library(ARDULIBS STATIC '>libs.short.txt.cmake
cat libs.short.readable.txt | xargs echo >> libs.short.txt.cmake
echo ')'>>libs.short.txt.cmake

echo review the contents of ./libs.short.txt.cmake in the current folder