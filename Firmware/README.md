### Compiler notes
We went through a bunch of trouble to install the newest GCC for MSP430. Useful weblinks:
  + http://www.ti.com/tool/msp430-gcc-opensource
  + http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSPGCC/latest/index_FDS.html

To program, we use mspdebug (again, the newest version cloned from Github), and also have to
link to the dynamic library libmsp430.so, which is found in the CCS directory (see Makefile).
  + 

Useful documentation on toolchain:
  + https://sites.google.com/site/yeltrow/msp430-gcc-opensource-on-ubuntu

And someone's Makefile prototype.
  + https://github.com/m-thu/msp430/blob/master/apa102/Makefile

_In the end, the issue with struct packing that prompted the upgrade (wanting to be able to
specify endian-ness), was not actually useful. The bug was that when we cast a char* to a
int16* it just **truncates the last bit of the address**, meaning that it will always be even,
and if we write to this pointer, it will actually overwrite the **byte before it** if it was
trying to be at an odd address._
