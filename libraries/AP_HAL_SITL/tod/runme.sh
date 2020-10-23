# http://theorangeduck.com/page/printing-stack-trace-mingw
rm test.exe 

x86_64-w64-mingw32-g++  -g test.cpp -ldbghelp -o test.exe 

wine test.exe

cp test.exe ~/Downloads/
