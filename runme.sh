./gnulib/gnulib-tool --import sys_socket bind sys_select send select socket setsockopt connect sendto recv recvfrom listen accept ; aclocal ; aclocal -I m4 ; autoheader ; automake --add-missing ; autoconf ; CXX=x86_64-w64-mingw32-g++ CC=x86_64-w64-mingw32-gcc ./configure --host=x86_64-w64-mingw32 ; make ; cp lib/libgnu.a build/sitl/lib/ ; CXX=x86_64-w64-mingw32-g++ CC=x86_64-w64-mingw32-gcc ./waf --target bin/arduplane -v 
