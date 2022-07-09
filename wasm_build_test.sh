#/bin/bash

#--------------------------------------------------
source ~/emsdk/emsdk_env.sh
export CXX='ccache em++'
export CC='ccache emcc'

# patch waflib:
( cd modules/waf ; patch --forward -p1 -r- < ../../em-waf.diff )

./waf configure board=sitl --debug --disable-scripting
./waf --target tests/test_notchfilter -v -v 

cp ./shell_minimal.html build/sitl/tests/index.html
cp build/sitl/tests/test_notchfilter build/sitl/tests/arduplane.js
( cd build/sitl/tests/ ; emrun --port 8000 . )
