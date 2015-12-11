#!/bin/bash

export CROSS_COMPILE=../arm-eabi-4.4.3/bin/arm-eabi-

export ARCH=arm

make neptuno_defconfig

make -j16
