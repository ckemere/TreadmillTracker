#!/usr/bin/env python

#%%
# NOTE: 

import time
import serial
import struct
import os
import argparse
import json
import datetime
from shutil import copy

from oscpy.client import OSCClient

### Maybe should add argcomplete for this program?

# GPIO IDs on Hat
GPIO_IDs = [16, 17, 18, 19, 24, 25, 26, 27, -1]

# Command-line arguments: computer settings
parser = argparse.ArgumentParser(description='Run simple linear track experiment.')
parser.add_argument('-P', '--serial-port', default='/dev/ttyS0',
                   help='TTY device for USB-serial interface (e.g., /dev/ttyUSB0 or COM10)')
parser.add_argument('-O', '--osc-port', type=int, default=12345, 
                   help='Serial port for OSC client')
parser.add_argument('--prefix', default='Data Log - ',
                   help='Prefix for output file - defaults to [Data Log - ]')
parser.add_argument('--param-file', default='defaults.json',
                    help='JSON file containing task parameters')
parser.add_argument('--output-dir', default='./',
                    help='Directory to write output file (defaults to cwd)')
args = parser.parse_args()
if not os.path.isdir(args.output_dir):
    os.mkdir(args.output_dir)
if not args.output_dir.endswith('/'):
    args.output_dir += '/'
print(args)

now = datetime.datetime.now()

# JSON parameters: task settings
with open(args.param_file) as f:
    d = json.loads(f.read())

    # Sound timing
    LickTimeout = d['Delay']['LickTimeout']
    PostDispenseDelay = d['Delay']['PostDispenseDelay']

    # Sound settings ([on_volume, off_volume])
    PinkNoiseOn = d['Sound']['PinkNoise']['OnVolume']
    PinkNoiseOff = d['Sound']['PinkNoise']['OffVolume']
    ToneCloudOn = d['Sound']['ToneCloud']['OnVolume']
    ToneCloudOff = d['Sound']['ToneCloud']['OffVolume']
    ToneOn = d['Sound']['Tone']['OnVolume']
    ToneOff = d['Sound']['Tone']['OffVolume']

    # GPIO configuration
    LeftPokeGPIO = d['GPIO']['LeftPoke']
    RightPokeGPIO = d['GPIO']['RightPoke']
    LeftLickGPIO = d['GPIO']['LeftLick']
    RightLickGPIO = d['GPIO']['RightLick']
    LeftDispenseGPIO = d['GPIO']['LeftDispense']
    RightDispenseGPIO = d['GPIO']['RightDispense']

# Save session parameters to output directory
new_fp = args.output_dir + 'params_' + now.strftime("%Y-%m-%d_%H%M")
copy(args.param_file, new_fp)

#%%
if False:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)

    def setGPIO(value=False, pin=17):
        GPIO.output(pin, value)



#%%

#%%
import csv

filename = '{}{}.txt'.format('Log', now.strftime("%Y-%m-%d %H%M"))

#%%
from enum import Enum, auto, unique
class MazeStates(Enum):
    NotLicked = auto()
    Licked = auto()

class SoundType(Enum):
    # Minimixer channels for each type of sound
    NoSound = 0
    PinkNoise = 1
    ToneCloud = 2
    Tone = 3

class Sounds:
    def __init__(self, WhichSound=SoundType.NoSound,
            OnVolume=0.0, OffVolume=-1000.0,
            OscPort=None):
        self.sound = WhichSound
        self.OnVolume = float(OnVolume)
        self.OffVolume = float(OffVolume)
        if OscPort is not None:
            self.oscC = OSCClient('127.0.0.1', int(OscPort))
        else:
            self.oscC = OSCClient('127.0.0.1', 12345)

    def Play(self):
        if self.sound is not SoundType.NoSound:
            self.oscC.send_message(b'/mixer/channel/set_gain',[int(self.sound.value), self.OnVolume])
            print('Playing ', self.sound.name)

    def Stop(self):
        if self.sound is not SoundType.NoSound:
            self.oscC.send_message(b'/mixer/channel/set_gain',[int(self.sound.value), self.OffVolume])
            print('Stopping ', self.sound.name)


#%%
class WellData:
    # State logic: store as integer type
    LeftLickMask = 2**(GPIO_IDs.index(LeftLickGPIO))
    # RightLickMask = 2**(GPIO_IDs.index(RightLickGPIO))

    # Serial data: store as binary string
    LeftDispenseMask = chr(2**(GPIO_IDs.index(LeftDispenseGPIO))).encode()
    #RightDispenseMask = chr(2**(GPIO_IDs.index(RightDispenseGPIO))).encode()


#%%


# Initialization

CurrentMazeState = MazeStates.NotLicked
PinkNoiseStim = Sounds(SoundType.PinkNoise, OnVolume=PinkNoiseOn, OffVolume=PinkNoiseOff, OscPort=args.osc_port)
ToneStim = Sounds(SoundType.Tone, OnVolume=ToneOn, OffVolume=ToneOff, OscPort=args.osc_port)

#%%
from subprocess import Popen, DEVNULL

# Dispense on both ends to prime
Well = WellData()

# with open(filename, 'w', newline='') as log_file:
    # writer = csv.writer(log_file)

print('Getting ready to start')
PinkNoiseStim.Stop()
time.sleep(1)
PinkNoiseStim.Stop()
PinkNoiseStim.Play()

DelayEnd = 0

#%%
################
# With the serial data stream, because the rate of transfer is so high, the input buffer will
# over flow quite rapidly. So the following chunk of code (which connects,  empties out the
# buffer and gets things aligned on individual messages) needs to be run as closely as possible
# to the actual start of data transfer!
print('Connecting to serial/USB interface {} and synchronizing.'.format(args.serial_port))

class SerialInterface():
    def __init__(self, SerialPort='/dev/ttyS0'):
        print('Connecting to serial/USB interface {} and synchronizing.'.format(SerialPort))
        self.serial = serial.Serial(port=SerialPort,
            baudrate = 256000,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.2
        )

        # Synchronize immediately after opening port!

        # We read in a large buffer of data and find which offset is the start of the packets
        K = 3 # This code works for 100 but not 1000. Maybe related to buffer size???
        self.MessageLen = 14
        x=self.serial.read(self.MessageLen*(K+1))
        assert(len(x) == self.MessageLen*(K+1))
        # print(x) # useful for debugging....
        # Find offset in this set
        index = 0
        while(True):
            print(x[index:])
            continueFlag = False
            for k in range(K):
                if ( x.index(b'E',index + k*self.MessageLen) - (k*self.MessageLen + index)) != 0:
                    continueFlag = True
            if continueFlag:
                index = index + 1
            else:
                break
            if (index > (self.MessageLen-1)):
                print('Reached end with bad index')
                assert(False)
                break

        print('Found index: {}'.format(index))

        x = self.serial.read(index) # read the last little bit of the bad block, and we are in sync!


    def read_data(self):
        x=self.serial.read(self.MessageLen)
        assert(len(x)==self.MessageLen)
        FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO  = struct.unpack('<cBLhlBx', x)
        # print('Flag: {}. Clocks: {}. Encoder: {}. Unwrapped: {}, GPIO: 0x{:08b}'.format( 
            # FlagChar, MasterTime, Encoder, UnwrappedEncoder, GPIO))
        assert(FlagChar == b'E')
        return FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO

    def send_byte(self,data):
        if data is not None:
            self.serial.write(data)



with open(args.output_dir + filename, 'w', newline='') as log_file:
    writer = csv.writer(log_file)
    Interface = SerialInterface(SerialPort=args.serial_port)
    ## every 2 ms happens:
    while(True):
        last_ts = time.time()
        FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO = Interface.read_data()
        writer.writerow([MasterTime, GPIO, Encoder, UnwrappedEncoder, last_ts])

        if (MasterTime % 1000) == 0:
            print('Heartbeat {} : 0x{:08b} '.format(MasterTime, GPIO))

        IsLicked = (GPIO & Well.LeftLickMask) == Well.LeftLickMask


        if CurrentMazeState == MazeStates.NotLicked:
            if (MasterTime > DelayEnd and IsLicked): # Time for a state transition!
                print("sending byte", Well.LeftDispenseMask)
                Interface.send_byte(Well.LeftDispenseMask)
                DelayEnd = MasterTime + PostDispenseDelay    # wait till pump finish dispense
                CurrentMazeState = MazeStates.Licked
        else:  # now mouse is licked, set a time out for mouse so he doesn't keep getting reward as he finish up drinking
            if (MasterTime > DelayEnd):  # this delayend refers to syringe pump triggering
                Interface.send_byte(b'\x00') # reset wells
                DelayEnd = MasterTime + LickTimeout
                CurrentMazeState = MazeStates.NotLicked


