#!/usr/bin/env python

#%%
import time
import serial
import struct


#%%
ser = serial.Serial(port='COM6',
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
K = 1 # This code works for 100 but not 1000. Maybe related to buffer size???
MessageLen = 9
x=ser.read(MessageLen*(K+1)) # read K+1 messages
assert(len(x) == MessageLen*(K+1)) # ensure K+1 messages received

# Find offset in this set
index = 0
while(True):
    print(x[index:])
    continueFlag = False

    # Iterate over all possible offsets {0, 1, .., K-1}. If correct offset,
    # then start byte (E) should occur every MessageLen bytes. If not,
    # increment index to try next offset
    for k in range(K):
        if x.index(b'E', index + k*MessageLen) != (index + k*MessageLen):
            continueFlag = True
    if continueFlag:
        index = index + 1
    else:
        break
    
    # All possible offsets tried without success
    if (index > (MessageLen-1)):
        print('Reached end with bad index')
        assert(False)
        break

# Print successful offset for synchronization if found
print('Found index: {}'.format(index))

x = ser.read(index) # read the last little bit of the bad block

#%%
print('Capturing data')
lastMasterTime = 0; # microcontroller clock
lastSystemTime = 0; # host computer clock
FirstTSCaptured = False;

with open('IntereventDataLatency.txt', 'w') as out_file:
  while(True):
      x=ser.read(MessageLen) # read one message
      if (len(x) == MessageLen):
          last_ts = time.time()
          FlagChar, MasterTime, Encoder, GPIO  = struct.unpack('>cLhBx', x)
          if FirstTSCaptured:
            #assert((MasterTime - lastMasterTime) == 2)
            out_file.write('{}\n'.format(last_ts - lastSystemTime))
          FirstTSCaptured = True;
          lastMasterTime = MasterTime;
          lastSystemTime = last_ts;

