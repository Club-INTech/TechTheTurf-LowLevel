#! /bin/bash

cd build
cmake ..
make -j4
picotool load -F pami-main.uf2
picotool reboot
