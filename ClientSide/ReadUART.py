#!/usr/bin/env python

#%%
import time
import serial
import struct


#%%
ser = serial.Serial(port='/dev/ttyUSB0',
 baudrate = 9600,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 bytesize=serial.EIGHTBITS,
 timeout=0.2
)
counter=0


#%%
print(struct.calcsize('<xHHix'))

#%%
lastMasterTime = 0;
while(True):
    x=ser.read(8)
    if (len(x) == 8):
        FlagChar, MasterTime, Encoder = struct.unpack('>cLhx', x)
        print('Flag: {}. Clocks: {}. Encoder: {}'.format( 
            FlagChar, MasterTime, Encoder))
        print('Elapsed: {}'.format(MasterTime - lastMasterTime))
        lastMasterTime = MasterTime;
