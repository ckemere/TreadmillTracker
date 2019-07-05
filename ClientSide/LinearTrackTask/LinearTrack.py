#!/usr/bin/env python

#%%
import time
import serial
import struct
import argparse

### Maybe should add argcomplete for this program?


parser = argparse.ArgumentParser(description='Run simple linear track experiment.')
parser.add_argument('-P', '--serial-port', default='/dev/ttyS0',
                   help='TTY device for USB-serial interface (e.g., /dev/ttyUSB0 or COM10)')
parser.add_argument('-O', '--osc-port', type=int, default=12345, 
                   help='Serial port for OSC client')
parser.add_argument('--prefix', default='Data Log - ',
                   help='Prefix for output file - defaults to [Data Log - ]')

args = parser.parse_args()

print(args)

#%%
if False:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)

    def setGPIO(value=False, pin=17):
        GPIO.output(pin, value)



#%%
ser = serial.Serial(port=args.serial_port,
 baudrate = 256000,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 bytesize=serial.EIGHTBITS,
 timeout=0.2
)


#%%
print('Synchronizing')
## Assume that buffer overflows have lost synchronization to 9 byte packets
#     So we read in a large buffer of data and find which offset is the start
#     of the packets
K = 5 # This code works for 100 but not 1000. Maybe related to buffer size???
MessageLen = 14
x=ser.read(MessageLen*(K+1))
assert(len(x) == MessageLen*(K+1))
print(x)
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
print(x)

#%%
from oscpy.client import OSCClient
oscC = OSCClient('127.0.0.1', args.osc_port)

#%%
import datetime
import csv
now = datetime.datetime.now()
filename = '{}{}.txt'.format('Log', now.strftime("%Y-%m-%d %H%M"))

#%%
from enum import Enum, auto, unique
class MazeStates(Enum):
    NotPoked = auto()
    PokedLeft = auto()
    PokedRight = auto()
    PokedEither = auto()

@unique
class PokeSubstates(Enum):
    NotPoked = auto()
    BetweenDispensingDelay = auto()
#    ToneOn = auto()
    PreDispenseToneDelay = auto()
#    Dispense = auto()
    PostDispenseToneDelay = auto()
#    ToneOff = auto()

class SoundType(Enum):
    # Minimixer channels for each type of sound
    NoSound = 0
    PinkNoise = 1
    ToneCloud = 2
    Tone = 3

class Sounds:
    def __init__(self, whichSound=SoundType.NoSound):
        self.is_playing = False
        self.sound = whichSound

    def Play(self):
        if self.sound is not SoundType.NoSound:
            if not self.is_playing:
                print(self.sound)
                oscC.send_message(b'/mixer/channel/set_gain',[int(self.sound.value), float(0.0)])
                print('Playing ', self.sound.name)
                self.is_playing = True

    def Stop(self):
        if self.sound is not SoundType.NoSound:
            if self.is_playing:
                oscC.send_message(b'/mixer/channel/set_gain',[int(self.sound.value), float(-1000.0)])
                print('Stopping ', self.sound.name)
                self.is_playing = False


#%%
class Wells:
    LeftPokeMask = 0x01
    RightPokeMask = 0x02

    def LeftDispense(self):
        print('Dispensing left well')
    def RightDispense(self):
        print('Dispensing right well')

#%%
FirstBetweenDispensingDelay = 50
PreDispenseToneDelay = 20
PostDispenseToneDelay = 80
SubsequentBetweenDispensingDelay = 500

# Initialization

CurrentMazeState = MazeStates.NotPoked
ValidNextMazeState = MazeStates.PokedEither 
PinkNoiseStim = Sounds(SoundType.PinkNoise)
ToneCloudStim = Sounds(SoundType.ToneCloud)
ToneStim = Sounds(SoundType.Tone)


#%%
from subprocess import Popen, DEVNULL


# Dispense on both ends to prime
Well = Wells()
Well.LeftDispense()
Well.RightDispense()

with Popen(['/usr/local/bin/jackminimix', '-a', '-p', '12345'],
        stdout=DEVNULL) as p_jackminimix:
    time.sleep(1)
    with Popen(['/usr/local/bin/sndfile-jackplay', '-l0', 
            '-a=minimixer:in1_right', 'pink_noise.wav'], stdout=DEVNULL, stderr=DEVNULL) as p_pinknoise, \
        Popen(['/usr/local/bin/sndfile-jackplay', '-l0', 
            '-a=minimixer:in2_right', 'tone_cloud_no_gating.wav'], stdout=DEVNULL,
            stderr=DEVNULL) as p_tone_cloud, \
        Popen(['/usr/local/bin/sndfile-jackplay', '-l0', 
            '-a=minimixer:in3_right', 'tone.wav'], stdout=DEVNULL,
            stderr=DEVNULL) as p_tone, \
        open(filename, 'w', newline='') as log_file:

        writer = csv.writer(log_file)
        print('Getting ready to start')
        time.sleep(5)
        PinkNoiseStim.Play()
        ToneCloudStim.Stop()
        ToneStim.Stop()

        CurrentPokeState = PokeSubstates.NotPoked
        DelayEnd = 0

        print('Starting Master Loop')
        while(True):
            x=ser.read(MessageLen)
            last_ts = time.time()
            assert(len(x)==MessageLen)
            FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO  = struct.unpack('<cBLhlBx', x)
            
            writer.writerow([MasterTime, GPIO, last_ts])

            #print('Flag: {}. Clocks: {}. Encoder: {}. Unwrapped: {}, GPIO: 0x{:08b}'.format( 
            #    FlagChar, MasterTime, Encoder, UnwrappedEncoder, GPIO))

            IsLeftWellPoked = (GPIO & Well.LeftPokeMask) > 0
            IsRightWellPoked = (GPIO & Well.RightPokeMask) > 0

            if (IsLeftWellPoked and IsRightWellPoked):
                raise Exception('Error: both Nose Pokes Activated')

            if CurrentMazeState in (MazeStates.PokedLeft, MazeStates.PokedRight):
                if not (IsLeftWellPoked or IsRightWellPoked): # Change in state
                    print('Poked to not')
                    ToneCloudStim.Stop()
                    ToneStim.Stop()
                    PinkNoiseStim.Play()
                    DelayEnd = 0
                    CurrentPokeState = PokeSubstates.NotPoked
                    if CurrentMazeState == MazeStates.PokedLeft:
                        ValidNextMazeState = MazeStates.PokedRight
                        print('Next state right')
                    elif CurrentMazeState == MazeStates.PokedRight:
                        ValidNextMazeState = MazeStates.PokedLeft
                        print('Next state left')
                    CurrentMazeState = MazeStates.NotPoked

                    continue
                else:
                    if (MasterTime > DelayEnd): # Time for a state transition!
                        if CurrentPokeState == PokeSubstates.BetweenDispensingDelay:
                            ToneStim.Play()
                            DelayEnd = MasterTime + PreDispenseToneDelay
                            CurrentPokeState = PokeSubstates.PreDispenseToneDelay
                        elif CurrentPokeState == PokeSubstates.PreDispenseToneDelay:
                            if IsLeftWellPoked:
                                Well.LeftDispense()
                            elif IsRightWellPoked:
                                Well.RightDispense()
                            else:
                                raise Exception('Error in dispense state configuration.')
                            DelayEnd = MasterTime + PostDispenseToneDelay
                            CurrentPokeState = PokeSubstates.PostDispenseToneDelay
                        elif CurrentPokeState == PokeSubstates.PostDispenseToneDelay:
                            ToneStim.Stop()
                            DelayEnd = MasterTime + SubsequentBetweenDispensingDelay
                            CurrentPokeState = PokeSubstates.BetweenDispensingDelay
                        else:
                            raise Exception('Error in PokeSubstates state configuration.')


            else: # NotPoked state
                if IsLeftWellPoked or IsRightWellPoked: # Gone from Not Poked to PokedLeft or PokedRight
                    if (ValidNextMazeState == MazeStates.PokedEither):
                        CurrentMazeState = MazeStates.PokedLeft if IsLeftWellPoked else MazeStates.PokedRight
                    elif IsLeftWellPoked and (ValidNextMazeState == MazeStates.PokedLeft):
                        CurrentMazeState = MazeStates.PokedLeft
                    elif IsRightWellPoked and (ValidNextMazeState == MazeStates.PokedRight):
                        CurrentMazeState = MazeStates.PokedRight
                    else:
                        continue # The animal tried to poke again in a non-rewarding well
                    print('Not to poked')
                    ValidNextMazeState = MazeStates.NotPoked
                    PinkNoiseStim.Stop()
                    ToneCloudStim.Play()
                    CurrentPokeState = PokeSubstates.BetweenDispensingDelay
                    DelayEnd = MasterTime + FirstBetweenDispensingDelay
                else:
                    pass # Still not poked!
