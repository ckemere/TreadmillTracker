#!/usr/bin/env python

#%%
import time
import serial
import struct
import argparse

### Maybe should add argcomplete for this program?


parser = argparse.ArgumentParser(description='Log experimental GPIO and wheel data from USB Treadmill Tracker interface.')
parser.add_argument('-P', '--port', default='/dev/ttyUSB0',
                   help='TTY device for USB-serial interface (e.g., /dev/ttyUSB0 or COM10)')
parser.add_argument('--prefix', default='Wheel and IO Data Log - ',
                   help='Prefix for output file - defaults to [Wheel and IO Data Log -]')
parser.add_argument('-f','--filename', help='Output file name')

args = parser.parse_args()


#%%
serialport = args.port
ser = serial.Serial(port=serialport,
 baudrate = 256000,
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
MessageLen = 14
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
import datetime
import csv


if args.filename is None:
    now = datetime.datetime.now()
    filename = '{}{}.txt'.format(args.prefix, now.strftime("%Y-%m-%d %H%M"))

print('Capturing data to {}'.format(filename))
lastMasterTime = 0
lastSystemTime = 0
FirstTSCaptured = False

with open(filename, 'w', newline='') as out_file:
  fieldnames = ['FlagChar', 'MasterTime', 'Encoder', 'UnwrappedEncoder', 'GPIO', 'SystemTime']
  writer = csv.writer(out_file)
  writer.writerow(fieldnames)
  while(True):
      x=ser.read(MessageLen)
      if (len(x) == MessageLen):
          last_ts = time.time()
          FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO  = struct.unpack('<cBLhlBx', x)
          if FirstTSCaptured:
            #out_file.write('{},{},{},{},{}\n'.format(FlagChar, MasterTime, Encoder, GPIO, last_ts))
            writer.writerow([FlagChar, MasterTime, Encoder, UnwrappedEncoder, GPIO, last_ts])

          FirstTSCaptured = True;
          out_file.flush()

