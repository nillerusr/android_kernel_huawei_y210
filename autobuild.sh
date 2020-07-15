#!/bin/sh

export CROSS_COMPILE=$PWD/gcc/bin/arm-eabi-
export ARCH=arm
export USE_CCACHE=1
export O=out

if ! [ -d gcc ]
then
mkdir gcc
wget https://android.googlesource.com/platform/prebuilt/+archive/6c89c6601a699707500003b782406482c4c3050b/linux-x86/toolchain/arm-eabi-4.4.3.tar.gz
tar xvf arm-eabi-4.4.3.tar.gz -C gcc
rm arm-eabi-4.4.3.tar.gz
fi

[ -d $O ] || mkdir $O

[ -f $O/.config ] || make O=$O cyanogenmod_y210_defconfig
make O=$O -j$(($(nproc --all)+1))

abootimg -u boot/boot.img -k $O/arch/arm/boot/zImage
cd boot
zip -9r ../kernel.zip *
