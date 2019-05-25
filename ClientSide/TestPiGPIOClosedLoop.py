#!/usr/bin/env python

#%%
import time
import serial
import struct
import pigpio


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

#%%
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.OUT)

#%%
def setGPIO(value=False, pin=16):
    GPIO.output(pin, value)

#%%
trigger = 0
lastValue = 0
lastMasterTime = 0;
with open('PIClosedLoop.txt', 'w') as out_file:
  while(True):
      x=ser.read(9)
      if (len(x) == 9):
          FlagChar, MasterTime, Encoder, GPIOData  = struct.unpack('>cLhBx', x)
          if trigger:
              if (lastValue == GPIOData):
                out_file.write('{}\n'.format(MasterTime - lastMasterTime))
                print(MasterTime, lastMasterTime)
                trigger = False
                lastMasterTime = MasterTime;
                  
          # out_file.write('Clocks: {}. GPIO: 0x{:08b}\n'.format( MasterTime, GPIOData))

          if ((MasterTime % 10) == 0): # Trigger a test every 100 ms
            trigger = True;
            if (lastValue == 0):
              setGPIO(1)
              lastValue = 1
            else:
              setGPIO(0)
              lastValue = 0

