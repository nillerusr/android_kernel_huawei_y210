#!/bin/sh
make O=out ARCH=arm CROSS_COMPILE=$PWD/gcc/bin/arm-eabi- $*
