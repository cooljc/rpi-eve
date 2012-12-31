evesrf
======

SRF Kernel module for Raspberry Pi EVE Alpha board.

To use this module you will need to disable spidev 
and prevent bcm2708.c loading spi devices. There
is a kernel patch in the ../patches directory.

Boiler plate code was taken from:
http://github.com/scottellis/spike
