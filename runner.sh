#!/bin/sh

spin -a $1
gcc -DMEMLIM=4000 -O2 -DHC4 -DXUSAFE -DNOREDUCE -w -o pan pan.c
./pan -m9000000  -a -f -c1 -N $2