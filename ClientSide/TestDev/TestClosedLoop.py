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
K = 2 # This code works for 100 but not 1000. Maybe related to buffer size???
MessageLen = 9
x=ser.read(MessageLen*(K+1))
assert(len(x) == MessageLen*(K+1))
# Find offset in this set
index = 0
while(True):
    print(x[index:])
    continueFlag = False
    for k in range(K):
        if ( x.index(b'E',index + k*MessageLen) - (k*MessageLen + index)) != 0:
            continueFlag = True
    if continueFlag:
        index = index + 1
    else:
        break
    if (index > (MessageLen-1)):
        print('Reached end with bad index')
        assert(False)
        break

print('Found index: {}'.format(index))

x = ser.read(index) # read the last little bit of the bad block


#%%
trigger = 0
lastValue = 0
lastMasterTime = 0;
with open('ClosedLoop.txt', 'w') as out_file:
  while(True):
      x=ser.read(9)
      if (len(x) == 9):
          FlagChar, MasterTime, Encoder, GPIO  = struct.unpack('>cLhBx', x)
          if (trigger == 1):
            out_file.write('{}\n'.format(MasterTime - lastMasterTime))
            print(MasterTime, lastMasterTime)
            trigger = 0
          lastMasterTime = MasterTime;

          if ((MasterTime % 100) == 0): # Trigger a test every 100 ms
            trigger = 1;
            if (lastValue == 0):
              ser.write(b'\xf0')
              lastValue = 1
            else:
              ser.write(b'\x0f')
              lastValue = 0

