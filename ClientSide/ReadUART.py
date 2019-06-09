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
print('Synchronizing')
## Assume that buffer overflows have lost synchronization to 9 byte packets
#     So we read in a large buffer of data and find which offset is the start
#     of the packets
K = 10 # This code works for 100 but not 1000. Maybe related to buffer size???
x=ser.read(9*(K+1))
# Find offset in this set
index = 0
while(True):
    continueFlag = False
    for k in range(K):
        if x[k+index] != 'E':
            continueFlag = True
    if continueFlag:
        index = index + 1
        # temp = x[index:index+k]
        # print('{}: {}'.format(index, x))
    else:
        break
    if (index > 8):
        break


x = ser.read(9-index) # read the last little bit of the bad block


#%%
lastMasterTime = 0;
lastSystemTime = 0;
while(True):
    x=ser.read(9)
    last_ts = time.time()
    if (len(x) == 9):
        FlagChar, MasterTime, Encoder, GPIO  = struct.unpack('>cLhBx', x)
        # FlagChar, GPIO, MasterTime, Encoder  = struct.unpack('>cBLhx', x)
        print('Flag: {}. Clocks: {}. Encoder: {}. GPIO: 0x{:08b}'.format( 
            FlagChar, MasterTime, Encoder, GPIO))
        print('Elapsed: {} ({})'.format(MasterTime - lastMasterTime, last_ts - lastSystemTime))
        lastMasterTime = MasterTime;
        lastSystemTime = last_ts;

