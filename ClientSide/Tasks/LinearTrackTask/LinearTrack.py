#!/usr/bin/env python

# Imports
import time
import struct
import os
import argparse
import json
import datetime
from shutil import copy
import numpy as np
import csv
from enum import Enum, auto, unique
from subprocess import Popen, DEVNULL
from Interface import SerialInterface, Sounds

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

# JSON parameters: task settings
with open(args.param_file) as f:
    d = json.loads(f.read())

    # General info
    mouseID = d['Info']['MouseID']
    sessionID = d['Info']['Session']
    GPIO_IDs = d['Info']['GPIO_IDs'] # GPIO IDs on Hat
    if not (GPIO_IDs[-1] == -1):
        # Add [-1] at end to avoid using unused GPIO logic
        GPIO_IDs.append(-1)

    # Sound timing
    FirstBetweenDispensingDelay = d['Delay']['FirstBetweenDispensingDelay']
    PreDispenseToneDelay = d['Delay']['PreDispenseToneDelay']
    PostDispenseDelay = d['Delay']['PostDispenseDelay']
    PostDispenseToneDelay = d['Delay']['PostDispenseToneDelay']
    SubsequentBetweenDispensingDelay = d['Delay']['SubsequentBetweenDispensingDelay']
    PokeEntryDelay = d['Delay']['PokeEntryDelay']
    PokeExitDelay = d['Delay']['PokeExitDelay']

    # Sound settings
    PinkNoiseParams = d['Sound']['PinkNoise']
    ToneCloudParams = d['Sound']['ToneCloud']
    ToneParams = d['Sound']['Tone']

    # GPIO configuration
    LeftPokeGPIO = d['GPIO']['LeftPoke']
    RightPokeGPIO = d['GPIO']['RightPoke']
    LeftLickGPIO = d['GPIO']['LeftLick']
    RightLickGPIO = d['GPIO']['RightLick']
    LeftDispenseGPIO = d['GPIO']['LeftDispense']
    RightDispenseGPIO = d['GPIO']['RightDispense']

    # Reward decay
    tau = d['Reward']['tau']
    r_0 = d['Reward']['r_0']
    R_0 = d['Reward']['R_0']

# Save session parameters and set up data logging
dateString = datetime.datetime.now().strftime("%Y-%m-%d-%H%M")
newFilename = '{}_d{:02d}_{}_params.json'.format(mouseID, sessionID, dateString)
newFilepath = args.output_dir + newFilename
copy(args.param_file, newFilepath)
logFilename = '{}_d{:02d}_{}_log.txt'.format(mouseID, sessionID, dateString)

# Define logic classes
class MazeStates(Enum):
    NotPoked = auto()
    PokedLeft = auto()
    PokedRight = auto()
    PokedEither = auto()

@unique
class PokeSubstates(Enum):
    NotPoked = auto() # task logic state
    Exiting = auto() # sensor state (for smoothing)
    Entering = auto() # sensor state (for smoothing)
    BetweenDispensingDelay = auto()
    PreDispenseToneDelay = auto()
    PostDispenseDelay = auto()
    PostDispenseToneDelay = auto()

class WellData:
    # State logic: store as integer type
    LeftPokeMask = 2**(GPIO_IDs.index(LeftPokeGPIO))
    RightPokeMask = 2**(GPIO_IDs.index(RightPokeGPIO))
    LeftLickMask = 2**(GPIO_IDs.index(LeftLickGPIO))
    RightLickMask = 2**(GPIO_IDs.index(RightLickGPIO))

    # Serial data: store as binary string
    LeftDispenseMask = struct.pack('<B', 2**(GPIO_IDs.index(LeftDispenseGPIO)))
    RightDispenseMask = struct.pack('<B', 2**(GPIO_IDs.index(RightDispenseGPIO)))

# Initialize behavior logic
Well = WellData()
CurrentMazeState = MazeStates.NotPoked
ValidNextMazeState = MazeStates.PokedEither 
CurrentPokeState = PokeSubstates.NotPoked
DelayEnd = 0
LastEnterTime = 0
LastExitTime = 0
RewardNumber = 0
n = np.arange(int(tau*r_0/R_0))
tReward = -tau*np.log(1.0 - (n*R_0)/(tau*r_0)) * 1000 # ms
tDelay = np.append(np.diff(tReward), np.inf)

# Initialize sounds
PinkNoiseStim = Sounds(Name='PinkNoise', 
                       Channel=PinkNoiseParams['Channel'], 
                       OnVolume=PinkNoiseParams['OnVolume'], 
                       OffVolume=PinkNoiseParams['OffVolume'], 
                       OscPort=args.osc_port)
ToneCloudStim = Sounds(Name='ToneCloud', 
                       Channel=ToneCloudParams['Channel'], 
                       OnVolume=ToneCloudParams['OnVolume'], 
                       OffVolume=ToneCloudParams['OffVolume'], 
                       OscPort=args.osc_port)
ToneStim      = Sounds(Name='Tone', 
                       Channel=ToneParams['Channel'], 
                       OnVolume=ToneParams['OnVolume'], 
                       OffVolume=ToneParams['OffVolume'], 
                       OscPort=args.osc_port)

# Play sounds
print('Getting ready to start')
PinkNoiseStim.Stop()
ToneCloudStim.Stop()
ToneStim.Play()
time.sleep(1)
PinkNoiseStim.Stop()
ToneStim.Stop()
ToneCloudStim.Play()
time.sleep(1)
ToneCloudStim.Stop()
ToneStim.Stop()
PinkNoiseStim.Play()

# With the serial data stream, because the rate of transfer is so high, the input buffer will
# over flow quite rapidly. So the following chunk of code (which connects,  empties out the
# buffer and gets things aligned on individual messages) needs to be run as closely as possible
# to the actual start of data transfer!
print('Connecting to serial/USB interface {} and synchronizing.'.format(args.serial_port))

with open(args.output_dir + logFilename, 'w', newline='') as log_file:
    writer = csv.writer(log_file)
    Interface = SerialInterface(SerialPort=args.serial_port)
    while(True):
        last_ts = time.time()
        FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO = Interface.read_data()
        isPoked = int(CurrentMazeState in (MazeStates.PokedLeft, MazeStates.PokedRight))
        writer.writerow([MasterTime, last_ts, GPIO, isPoked])

        if (MasterTime % 1000) == 0:
            print('Heartbeat {} : 0x{:08b} '.format(MasterTime, GPIO))

        IsLeftWellPoked = (GPIO & Well.LeftPokeMask) == Well.LeftPokeMask
        IsRightWellPoked = (GPIO & Well.RightPokeMask) == Well.RightPokeMask

        if (IsLeftWellPoked and IsRightWellPoked):
            print('Flag: {}. Clocks: {}. Encoder: {}. Unwrapped: {}, GPIO: 0x{:08b}'.format(FlagChar, MasterTime, Encoder, UnwrappedEncoder, GPIO))
            raise Exception('Error: both Nose Pokes Activated')

        if CurrentMazeState in (MazeStates.PokedLeft, MazeStates.PokedRight):
            # Left Poke in Poked MazeState
            if not (IsLeftWellPoked or IsRightWellPoked):
                # Change poke state and set timer until NotPoked
                if CurrentPokeState != PokeSubstates.Exiting: 
                    ToneStim.Stop()
                    CurrentPokeState = PokeSubstates.Exiting
                    DelayEnd = MasterTime + PokeExitDelay
                # Change from poked to not if delay surpasssed
                elif (MasterTime > DelayEnd): # Change in state
                    print('Poked to not')
                    ToneCloudStim.Stop()
                    ToneStim.Stop()
                    PinkNoiseStim.Play()
                    DelayEnd = 0
                    RewardNumber = 0
                    CurrentPokeState = PokeSubstates.NotPoked
                    if CurrentMazeState == MazeStates.PokedLeft:
                        ValidNextMazeState = MazeStates.PokedRight
                        print('Next state right')
                    elif CurrentMazeState == MazeStates.PokedRight:
                        ValidNextMazeState = MazeStates.PokedLeft
                        print('Next state left')
                    CurrentMazeState = MazeStates.NotPoked

                continue

            # Reset to post-dispense state if re-entered before delay
            elif CurrentPokeState == PokeSubstates.Exiting:
                DelayEnd = MasterTime + SubsequentBetweenDispensingDelay
                CurrentPokeState = PokeSubstates.BetweenDispensingDelay
            
            # Check for transition to new substate
            elif (MasterTime > DelayEnd): # Time for a state transition!
                if CurrentPokeState == PokeSubstates.BetweenDispensingDelay:
                    ToneStim.Play()
                    DelayEnd = MasterTime + PreDispenseToneDelay
                    CurrentPokeState = PokeSubstates.PreDispenseToneDelay
                elif CurrentPokeState == PokeSubstates.PreDispenseToneDelay:
                    if IsLeftWellPoked:
                        print("sending byte", Well.LeftDispenseMask)
                        Interface.send_byte(Well.LeftDispenseMask)
                    elif IsRightWellPoked:
                        print("sending byte", Well.RightDispenseMask)
                        Interface.send_byte(Well.RightDispenseMask)
                    else:
                        raise Exception('Error in dispense state configuration.')
                    DelayEnd = MasterTime + PostDispenseDelay
                    CurrentPokeState = PokeSubstates.PostDispenseDelay
                elif CurrentPokeState == PokeSubstates.PostDispenseDelay:
                    Interface.send_byte(b'\x00') # reset wells
                    DelayEnd = MasterTime + PostDispenseToneDelay
                    CurrentPokeState = PokeSubstates.PostDispenseToneDelay
                elif CurrentPokeState == PokeSubstates.PostDispenseToneDelay:
                    ToneStim.Stop()
                    DelayEnd = MasterTime + tDelay[RewardNumber]
                    RewardNumber += 1
                    CurrentPokeState = PokeSubstates.BetweenDispensingDelay
                else:
                    raise Exception('Error in PokeSubstates state configuration.')


        else: # NotPoked state
            if IsLeftWellPoked or IsRightWellPoked: # Gone from Not Poked to PokedLeft or PokedRight
                if CurrentPokeState != PokeSubstates.Entering:
                    CurrentPokeState = PokeSubstates.Entering
                    DelayEnd = MasterTime + PokeEntryDelay
                elif (MasterTime > DelayEnd):
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
            elif CurrentPokeState != PokeSubstates.NotPoked: # was Entering but now left well
                CurrentPokeState = PokeSubstates.NotPoked
            else:
                pass # Still not poked!
