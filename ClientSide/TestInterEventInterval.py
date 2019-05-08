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
K = 100 # This code works for 100 but not 1000. Maybe related to buffer size???
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
print('Capturing data')
lastMasterTime = 0;
lastSystemTime = 0;
FirstTSCaptured = False;

with open('IntereventDataLatency.txt', 'w') as out_file:
  while(True):
      x=ser.read(9)
      if (len(x) == 9):
          last_ts = time.time()
          FlagChar, MasterTime, Encoder, GPIO  = struct.unpack('>cLhBx', x)
          if FirstTSCaptured:
            assert((MasterTime - lastMasterTime) == 10)
            out_file.write('{}\n'.format(last_ts - lastSystemTime))
          FirstTSCaptured = True;
          lastMasterTime = MasterTime;
          lastSystemTime = last_ts;

