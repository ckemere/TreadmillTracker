#!/usr/bin/env python

#%%
import time
import serial
import struct


#%%
ser = serial.Serial(port='/dev/ttyUSB0',
 baudrate = 256000,
 # baudrate = 9600,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 bytesize=serial.EIGHTBITS,
 timeout=0.2
)
counter=0


#%%
lastMasterTime = 0;
while(True):
    x=ser.read(9)
    if (len(x) == 9):
        FlagChar, MasterTime, Encoder, GPIO  = struct.unpack('>cLhBx', x)
        # FlagChar, GPIO, MasterTime, Encoder  = struct.unpack('>cBLhx', x)
        print('Flag: {}. Clocks: {}. Encoder: {}. GPIO: 0x{:08b}'.format( 
            FlagChar, MasterTime, Encoder, GPIO))
        print('Elapsed: {}'.format(MasterTime - lastMasterTime))
        lastMasterTime = MasterTime;

