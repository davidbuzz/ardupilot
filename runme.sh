#!/bin/bash

./gnulib/gnulib-tool --import sys_socket bind sys_select send select socket setsockopt connect open close sendto recv recvfrom listen accept unistd gethostname getsockname gettimeofday fcntl-h  fcntl nonblocking 
#...consider the ‘nonblocking’ module, that is an indicator that all I/O functions should handle non-blocking file descriptors – something that, is not enabled by default.
#unused things from previous line: termios stat / read write open 

# target and rebuild gnulib for ming:
 aclocal 
 aclocal -I m4 
 autoheader 
 automake --add-missing 
 autoconf 
CXX=x86_64-w64-mingw32-g++ CC=x86_64-w64-mingw32-gcc ./configure --host=x86_64-w64-mingw32  CPPFLAGS="-DGNULIB_NAMESPACE=gnulib"
sleep 1
tail -5 lib/config.h
echo "#define GNULIB_NAMESPACE gnulib" >> ./lib/config.h
echo "" >> ./lib/config.h
tail -5 lib/config.h
make 
tail -5 lib/config.h
echo "#define GNULIB_NAMESPACE  gnulib" >> ./lib/config.h
echo "" >> ./lib/config.h
tail -5 lib/config.h
make 
tail -5 lib/config.h
sleep 1
#exit

# copy gnulib binary to where ming ardupilot can link against it:
 mkdir -p build/sitl/lib/ 
 cp lib/libgnu.a build/sitl/lib/ 

sleep 1
# build ming ardupilot
 CXX=x86_64-w64-mingw32-g++ CC=x86_64-w64-mingw32-gcc ./waf configure 
 CXX=x86_64-w64-mingw32-g++ CC=x86_64-w64-mingw32-gcc ./waf --target bin/arduplane 
#-v
