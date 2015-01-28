#!/bin/bash

export ARCH=arm

export CROSS_COMPILE=arm-linux-gnueabihf-

make imx_v7_udoo_defconfig
LOADADDR=0x10008000 make uImage 
make imx6q-udoo.dtb

echo "Done. You can load kernel and dtb with:"
echo "tftpboot 0x10800000 uImage; tftp 0x13000000 imx6q-udoo.dtb; bootm 0x10800000 - 0x13000000"

