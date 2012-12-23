tmp10x
======

Simple kernel module for reading temperature from TI-TMP100.

Loading Module
==============

modprobe i2c-dev
insmod tmp10x.ko
echo tmp10x 0x48 > /sys/class/i2c-adapter/i2c-0/new_device

