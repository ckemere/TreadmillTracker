#!/usr/bin/env python

#%%
import time
import serial
import struct


#%%
ser = serial.Serial(port='/dev/ttyAMA0',
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

from oscpy.client import OSCClient
oscC = OSCClient('127.0.0.1', 13724)



soundState = False
oldSoundState = False

while(True):
  x=ser.read(9)
  if (len(x) == 9):
      FlagChar, MasterTime, Encoder, GPIOData  = struct.unpack('>cLhBx', x)

      print(MasterTime, Encoder)

      if (Encoder > 0) & (Encoder < 200):
          soundState = True
      else:
          soundState = False

      if (soundState != oldSoundState):
          oldSoundState = soundState
          if soundState:
              oscC.send_message(b'/mixer/channel/set_gain',[int(1), float(0.0)])
          else:
              oscC.send_message(b'/mixer/channel/set_gain',[int(1), float(-60.0)])



