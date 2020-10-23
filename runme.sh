#!/bin/bash

#./waf clean

./gnulib/gnulib-tool --import sys_socket bind sys_select send select socket setsockopt connect close sendto recv recvfrom listen accept unistd gethostname getsockname gettimeofday fcntl-h  fcntl nonblocking 
 aclocal 
 aclocal -I m4 
 autoheader 
 automake --add-missing 
 autoconf 
 CXX=x86_64-w64-mingw32-g++ CC=x86_64-w64-mingw32-gcc ./configure --host=x86_64-w64-mingw32 
 make 
 mkdir -p build/sitl/lib/ 
 cp lib/libgnu.a build/sitl/lib/ 
 CXX=x86_64-w64-mingw32-g++ CC=x86_64-w64-mingw32-gcc ./waf configure 
 CXX=x86_64-w64-mingw32-g++ CC=x86_64-w64-mingw32-gcc ./waf --target bin/arduplane -v 

# if u get errors like...type_size:655: unknown type 12
# adding a -w seems to make it go away.
#wine64 build/sitl/bin/arduplane.exe --model plane
