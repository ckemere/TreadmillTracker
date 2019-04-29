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
trigger = 0
lastValue = 0
lastMasterTime = 0;
while(True):
    x=ser.read(9)
    if (len(x) == 9):
        FlagChar, MasterTime, Encoder, GPIO  = struct.unpack('>cLhBx', x)
        # print('Flag: {}. Clocks: {}. Encoder: {}. GPIO: 0x{:08b}'.format( 
            # FlagChar, MasterTime, Encoder, GPIO))
        if (trigger == 1):
          print('{}'.format(MasterTime - lastMasterTime))
          trigger = 0
        lastMasterTime = MasterTime;

        if ((MasterTime % 500) == 0):
          trigger = 1;
          if (lastValue == 0):
            ser.write(b'\xf0')
            lastValue = 1
          else:
            ser.write(b'\x0f')
            lastValue = 0

