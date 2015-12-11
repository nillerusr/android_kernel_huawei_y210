#!/bin/bash

export ARCH=arm

export CROSS_COMPILE=~/android/toolchain/arm-eabi-4.4.3/bin/arm-eabi-

make cyanogenmod_y210_defconfig

make -j16
