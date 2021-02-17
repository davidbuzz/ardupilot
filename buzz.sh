cd ~/ardupilot
rm -rf build/
./waf configure --board=esp32buzz --debug
./waf AP_Periph
cp -rp  build2_modules_libcanard build/modules/libcanard/
./waf AP_Periph --upload
