#!/usr/bin/env python

#%%
# NOTE: v2.1.1. 3 different Tones (3kHz, 6kHz, 12kHz) are played based on animal's position on the virtual track. 
#       ramping volume depend on a parameter named "peak_volume" describing how steep the ramping function
#       should be (default 13). Taking care the max peakVolume or OnVolume should not exceed -90dB and 90dB.
# Features: 
#     sound logic that is controlled only by linear_pos
#     pump logic controlled by PumpOn and PumpOffTime, so each time the pump is triggered, it must reset after 100ms regardless of animal's pos
#     peak_volume is constant number regardless of different tone frequencies
#     max_reward_times controls the max number of reward it can get within one single lap

import time
import serial
import struct
import os
import argparse
import json
import datetime
from shutil import copy
from oscpy.client import OSCClient
import numpy as np

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
parser.add_argument('--param-file', default='defaults.json',   ####### changed default!
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
    PeakTone1 = d['Sound']['Tone1']['peakVolume']
    ToneOff1 = d['Sound']['Tone1']['OffVolume']
    PeakTone2 = d['Sound']['Tone2']['peakVolume']
    ToneOff2 = d['Sound']['Tone2']['OffVolume']
    PeakTone3 = d['Sound']['Tone3']['peakVolume']
    ToneOff3 = d['Sound']['Tone3']['OffVolume']

    # GPIO configuration
    LeftPokeGPIO = d['GPIO']['LeftPoke']
    RightPokeGPIO = d['GPIO']['RightPoke']
    LeftLickGPIO = d['GPIO']['LeftLick']
    RightLickGPIO = d['GPIO']['RightLick']
    LeftDispenseGPIO = d['GPIO']['LeftDispense']
    RightDispenseGPIO = d['GPIO']['RightDispense']

    # Maze parameters
    ZoneDistance = d['Info']['ZoneDistance'] #cm, both silent zone and tone zone
    SilentZoneDelay = d['Info']['SilentZoneDelay'] #cm, distance after tone ends, before rewards given
    ValidRewardZone = d['Info']['ValidRewardZone'] #cm, reward zone distance
    zonebuffer = d['Info']['zonebuffer']  #cm prezone and postzone buffer distance
    max_reward_times = d['Info']['max_reward_times']  # number of reward the animal can get in one lap

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
import csv

filename = '{}{}.txt'.format('Log', now.strftime("%Y-%m-%d %H%M"))

#%%
from enum import Enum, auto, unique
class MazeStates(Enum):
    NotLicked = auto()
    Licked = auto()
    ToneZone1 = auto()
    ToneZone2 = auto()
    ToneZone3 = auto()
    SilentZones = auto()


@unique
class ZoneSubstates(Enum):
    BetweenZone = auto()
    PostZone = auto()


class SoundType(Enum):
    # Minimixer channels for each type of sound
    NoSound = 0
    PinkNoise = 1
    # ToneCloud = 2
    Tone1 = 2
    Tone2 = 3
    Tone3 = 4

class Sounds:
    def __init__(self, WhichSound=SoundType.NoSound, 
            OffVolume=-1000.0, OscPort=None):
        self.sound = WhichSound
        self.OffVolume = float(OffVolume)
        self.CurrentVolume = float(OffVolume)
        # self.peakVolume = float(peakVolume)
     
        if OscPort is not None:
            self.oscC = OSCClient('127.0.0.1', int(OscPort))
        else:
            self.oscC = OSCClient('127.0.0.1', 12345)

        self.Stop()

    def Stop(self):
        if self.sound is not SoundType.NoSound:
            if (self.CurrentVolume != self.OffVolume):
                self.CurrentVolume = self.OffVolume
                self.oscC.send_message(b'/mixer/channel/set_gain',[int(self.sound.value), self.OffVolume])
                # print('Stopping ', self.sound.name)

    def ChangePlay(self, volume):
        if self.sound is not SoundType.NoSound:
            if( self.CurrentVolume != volume ):
                if (volume == self.OffVolume):
                    self.CurrentVolume = self.OffVolume
                    self.oscC.send_message(b'/mixer/channel/set_gain',[int(self.sound.value), self.OffVolume])
                else:
                    self.oscC.send_message(b'/mixer/channel/set_gain',[int(self.sound.value), volume])
                    self.CurrentVolume = volume
                # self.oscC.send_message(b'/mixer/channel/set_gain',[int(self.sound.value), self.OnVolume * scale(distance)])


def distance_to_volume(distance, ZoneDistance, PeakVolume, OffVolume):
    """ input: linear_pos of the mouse running on the wheel, assume linear_pos = 0cm.
               change "peak_volume" to control the steepness of the volume scale function
        output: scaled up or down volume based on the distance. A triangle consist of y = d/(zonedistance/2)
                and y = - 1/7.5 *d + 2; 1 = peak volume. 7.5 = zonedistance/2  
    """
    if 0 <= distance  < ZoneDistance/2:
        volume = lambda d: PeakVolume * d / (ZoneDistance/2)
        return volume(distance)
    elif ZoneDistance/2 <= distance  <= ZoneDistance:
        volume = lambda d: - PeakVolume * d / (ZoneDistance/2) + 2*PeakVolume
        return volume(distance)
    else:
        # print('return same volume scale ---')
        return OffVolume
        #return 1  # do not change the volume scale otherwise
    

#%%
class WellData:
    # State logic: store as integer type
    LeftLickMask = 2**(GPIO_IDs.index(LeftLickGPIO))
    # RightLickMask = 2**(GPIO_IDs.index(RightLickGPIO))

    # Serial data: store as binary string
    LeftDispenseMask = bytes([2**(GPIO_IDs.index(LeftDispenseGPIO))])
    print(LeftDispenseMask)
    #RightDispenseMask = chr(2**(GPIO_IDs.index(RightDispenseGPIO))).encode()

#%%
# Initialization

CurrentLickState = MazeStates.NotLicked
currentMazeState = MazeStates.SilentZones
currentZoneState = ZoneSubstates.PostZone

PinkNoiseStim = Sounds(SoundType.PinkNoise, OffVolume=PinkNoiseOff, OscPort=args.osc_port)
ToneStim1 = Sounds(SoundType.Tone1, OffVolume=ToneOff1, OscPort=args.osc_port)
ToneStim2 = Sounds(SoundType.Tone2, OffVolume=ToneOff2, OscPort=args.osc_port)
ToneStim3 = Sounds(SoundType.Tone3, OffVolume=ToneOff3, OscPort=args.osc_port)

#%%
from subprocess import Popen, DEVNULL

# Dispense on both ends to prime
Well = WellData()

# with open(filename, 'w', newline='') as log_file:
    # writer = csv.writer(log_file)

print('Getting ready to start')
PinkNoiseStim.Stop()
ToneStim1.ChangePlay(PeakTone1)
time.sleep(1)
PinkNoiseStim.Stop()
ToneStim1.Stop()
ToneStim2.ChangePlay(PeakTone2)
time.sleep(1)
ToneStim1.Stop()
ToneStim2.Stop()
ToneStim3.ChangePlay(PeakTone3)
time.sleep(1)
ToneStim1.Stop()
ToneStim2.Stop()
ToneStim3.Stop()
PinkNoiseStim.ChangePlay(PinkNoiseOn)

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

#%%
#----------------------- parameters --------------
d = 20.2 #cm diameter of the physical wheel; 150cm
VirtualTrackDistance = ZoneDistance * 6 #cm
count = 0

#%%

with open(args.output_dir + filename, 'w', newline='') as log_file:
    writer = csv.writer(log_file)
    Interface = SerialInterface(SerialPort=args.serial_port)
    ## initiate encoder value ##
    FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO = Interface.read_data()
    initialUnwrappedencoder = UnwrappedEncoder 
    print("initial unwrapped encoder value : ", UnwrappedEncoder)
    Interface.send_byte(b'\x00') # reset wells
    PumpOn = False

    ### Initialize some delay flags to be something close to zero
    PumpOffTime = MasterTime
    DelayEnd = MasterTime

    ## every 2 ms happens:
    while(True):
        # last_ts = time.time()
        last_ts = time.monotonic()   # to match with miniscope timestamps (which is written in msec, here is sec)
        FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO = Interface.read_data()
        writer.writerow([MasterTime, GPIO, Encoder, UnwrappedEncoder, last_ts])

        if (MasterTime % 3000) == 0:
            print('Heartbeat {} : 0x{:08b} '.format(MasterTime, GPIO))

        #######################################################################################
        ### Logic to turn reward pump off if something happened
        if ((PumpOn == True) and (MasterTime > PumpOffTime)):  #Pumpofftime = current time when the pump was on + how long to keep the pump on for
            Interface.send_byte(b'\x00') # reset wells
            PumpOn = False
        #######################################################################################
        # Get linear position
        linear_pos = (UnwrappedEncoder - initialUnwrappedencoder) * d /4096 * np.pi 

        #######################################################################################
        ### Logic of virtual reality sounds
        volume1, volume2, volume3 = (distance_to_volume(linear_pos % VirtualTrackDistance- ZoneDistance, ZoneDistance, PeakTone1, ToneOff1),
                 distance_to_volume(linear_pos % VirtualTrackDistance- ZoneDistance*3, ZoneDistance, PeakTone2, ToneOff2),
                 distance_to_volume(linear_pos % VirtualTrackDistance- ZoneDistance*5, ZoneDistance, PeakTone3, ToneOff3))
        
        ToneStim1.ChangePlay(volume1)
        ToneStim2.ChangePlay(volume2)
        ToneStim3.ChangePlay(volume3)

        if (volume1 == ToneOff1) and (volume2 == ToneOff2) and (volume3 == ToneOff3):
            PinkNoiseStim.ChangePlay(PinkNoiseOn)
        else:
            PinkNoiseStim.Stop()
        #######################################################################################

        if ZoneDistance - zonebuffer < linear_pos % VirtualTrackDistance < ZoneDistance: #prezone
            if (currentMazeState == MazeStates.SilentZones and currentZoneState == ZoneSubstates.PostZone):
                currentZoneState = ZoneSubstates.BetweenZone
                print('post silent zone 1, play tone 1')

        elif ZoneDistance <= linear_pos % VirtualTrackDistance <= ZoneDistance + ZoneDistance: #tone 1
            if (currentMazeState == MazeStates.SilentZones and currentZoneState == ZoneSubstates.BetweenZone):
                currentMazeState = MazeStates.ToneZone1
            # if (MasterTime % 500) == 0:  # check and update the volume based on position every 300 ms.
                # volume = distance_to_volume(linear_pos % VirtualTrackDistance - ZoneDistance, ZoneDistance)
                # print("update volume info, ", volume)

        elif ZoneDistance*2 < linear_pos % VirtualTrackDistance < ZoneDistance*2 +zonebuffer: #postzone 
            if (currentMazeState == MazeStates.ToneZone1 and currentZoneState == ZoneSubstates.BetweenZone):
                currentZoneState = ZoneSubstates.PostZone
                print('post tone zone 1, silent 2')

        elif ZoneDistance*2 +zonebuffer <= linear_pos % VirtualTrackDistance <= ZoneDistance*3 -zonebuffer: #silent 2
            if (currentMazeState == MazeStates.ToneZone1 and currentZoneState == ZoneSubstates.PostZone):
                currentMazeState = MazeStates.SilentZones

        elif ZoneDistance*3 -zonebuffer < linear_pos % VirtualTrackDistance < ZoneDistance*3: #prezone 
            if (currentMazeState == MazeStates.SilentZones and currentZoneState == ZoneSubstates.PostZone):
                currentZoneState = ZoneSubstates.BetweenZone
                print('post silent zone 2, play tone 2')

        elif ZoneDistance*3 <= linear_pos % VirtualTrackDistance <= ZoneDistance*4: #tone 2
            if (currentMazeState == MazeStates.SilentZones and currentZoneState == ZoneSubstates.BetweenZone):
                currentMazeState = MazeStates.ToneZone2
            
        elif ZoneDistance*4 < linear_pos % VirtualTrackDistance < ZoneDistance*4 +zonebuffer: #postzone 
            if (currentMazeState == MazeStates.ToneZone2 and currentZoneState == ZoneSubstates.BetweenZone):
                currentZoneState = ZoneSubstates.PostZone
                print('post tone zone 2, entering free reward zone, silent 3')
                
        elif ZoneDistance*4 +zonebuffer <= linear_pos % VirtualTrackDistance <= ZoneDistance*5 -zonebuffer: #reward zone
            if (currentMazeState == MazeStates.ToneZone2 and currentZoneState == ZoneSubstates.PostZone):
                currentMazeState = MazeStates.SilentZones

            if ZoneDistance*4 + SilentZoneDelay <= linear_pos % VirtualTrackDistance <= ZoneDistance*4 + SilentZoneDelay + ValidRewardZone:
                if (MasterTime % 1000) == 0:
                    print('in operant REWARD zone...')
                ## operant condition, check if the mouse licked, if he did, then dispense water
                ## currenly he has to wait for LickTimeout amount of time (3s?) before he receives next water
                IsLicked = (GPIO & Well.LeftLickMask) == Well.LeftLickMask
                if CurrentLickState == MazeStates.NotLicked:
                    if (MasterTime > DelayEnd and IsLicked and count < max_reward_times): # Time for a state transition!
                        print("sending byte", Well.LeftDispenseMask)
                        Interface.send_byte(Well.LeftDispenseMask)
                        PumpOffTime = MasterTime + PostDispenseDelay    # wait till pump finish dispense
                        PumpOn = True
                        count += 1
                        DelayEnd = MasterTime + LickTimeout  # 1sec lick time out
                
        elif ZoneDistance*5 -zonebuffer < linear_pos % VirtualTrackDistance < ZoneDistance*5: #prezone 
            if (currentMazeState == MazeStates.SilentZones and currentZoneState == ZoneSubstates.PostZone):
                currentZoneState = ZoneSubstates.BetweenZone
                print('post reward location, play tone 3')
                count = 0

        elif ZoneDistance*5 <= linear_pos % VirtualTrackDistance <= ZoneDistance*6: #tone 3
            if (currentMazeState == MazeStates.SilentZones and currentZoneState == ZoneSubstates.BetweenZone):
                currentMazeState = MazeStates.ToneZone3

        elif 0 < linear_pos % VirtualTrackDistance < zonebuffer: #postzone, beginning of silent zone 1
            if (currentMazeState == MazeStates.ToneZone3 and currentZoneState == ZoneSubstates.BetweenZone):
                currentZoneState = ZoneSubstates.PostZone
                print('post tone zone 3, silent 1')

        elif zonebuffer <= linear_pos % VirtualTrackDistance <= ZoneDistance -zonebuffer: #silent 1
            if (currentMazeState == MazeStates.ToneZone3 and currentZoneState == ZoneSubstates.PostZone):
                currentMazeState = MazeStates.SilentZones
                