# MSP430 toolchain
CCPREFIX=/mnt/d/Development/Ti/msp430-gcc-7.3.2.154_linux64
INCPREFIX=/mnt/d/Development/Ti/msp430-gcc-support-files
TILIBPATH=/home/ckemere/ti/ccs900/ccs/ccs_base/DebugServer/drivers/
# mspdebug
MSPDEBUG=mspdebug

DEVICE=msp430g2553

PROJECT=treadmill

SRC=main.c Quadrature.c uart.c

OBJ=$(SRC:.c=.o)

CC=$(CCPREFIX)/bin/msp430-elf-gcc
# -Wextra removed to get rid of -Wold-style-declaration warnings
CFLAGS=-mmcu=$(DEVICE) -I. -I $(INCPREFIX)/include -ggdb -g3 -Os -pipe -pedantic -Wall# -Wextra 
LFLAGS=-L $(INCPREFIX)/include -lgcc

all: $(PROJECT)

$(PROJECT): $(OBJ)
	$(CC) -mmcu=$(DEVICE) $(LFLAGS) -o $(PROJECT) $(OBJ)

flash: $(PROJECT)
	LD_LIBRARY_PATH=$(TILIBPATH) $(MSPDEBUG) tilib 'prog $(PROJECT)'

clean:
	rm -f *.o
	rm -f $(PROJECT)

.PHONY: all flash clean
