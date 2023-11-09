#! /bin/bash

cd build
cmake ..
make -j
picotool load -F pami-main.uf2
picotool reboot
