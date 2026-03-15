#!/bin/bash
cd modules/ChibiOS
git format-patch HEAD~1
cp *.patch ..
cd ..
echo "please commit patch files in ./modules as a local way to track our Chibios changes"
rm modules/ChibiOS/*.patch