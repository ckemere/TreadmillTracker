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
import os
import argparse
import yaml
import datetime
import numpy as np

### Maybe should add argcomplete for this program?

# GPIO IDs on IO board
GPIO_IDs = [16, 17, 18, 19, 24, 25, 26, 27, -1]

# Command-line arguments: computer settings
# Command-line arguments: computer settings
parser = argparse.ArgumentParser(description='Run simple linear track experiment.')
parser.add_argument('-P', '--serial-port', default='/dev/ttyS0',
                   help='TTY device for USB-serial interface (e.g., /dev/ttyUSB0 or COM10)')
parser.add_argument('--param-file', default='defaults.yaml',  
                    help='YAML file containing task parameters')
parser.add_argument('--output-dir', default='./',
                    help='Directory to write output file (defaults to cwd)')
args = parser.parse_args()
if not os.path.isdir(args.output_dir):
    os.mkdir(args.output_dir)
if not args.output_dir.endswith('/'):
    args.output_dir += '/'
print(args)


# YAML parameters: task settings
with open(args.param_file, 'r') as f:
    Config = yaml.safe_load(f)

print('Normalizing stimuli:')
StimuliList = Config['AuditoryStimuli']['StimuliList']
for stimulus_name, stimulus in StimuliList.items(): 
    print(' - ',stimulus_name)
    for key, config_item in Config['AuditoryStimuli']['Defaults'].items():
        if key not in stimulus:
            stimulus[key] = config_item
        elif isinstance(config_item, dict):
            for subkey, sub_config_item in config_item.items():
                if subkey not in stimulus[key]:
                    stimulus[key][subkey] = sub_config_item


#----------------------- parameters --------------
VirtualTrackLength = Config['Maze']['Length'] #cm
d = Config['Maze']['WheelDiameter'] #cm diameter of the physical wheel; 150cm
count = 0


#%%
from RenderTrack import RenderTrack

visualization = RenderTrack(d/2)

from SoundStimulus import SoundStimulus

SoundStimuliList = []

for stimulus_name, stimulus in StimuliList.items():

    filename = stimulus['Filename']
    if Config['Preferences']['AudioFileDirectory']:
        filename = os.path.join(Config['Preferences']['AudioFileDirectory'], filename)
    print('Loading: {}'.format(filename))
    SoundStimuliList.append(SoundStimulus(filename=filename))

    if stimulus['Type'] == 'Background':
        visualization.add_zone(0, np.pi*2, fillcolor=stimulus['Color'], width=0.5, alpha=0.75)
        SoundStimuliList[-1].change_gain(stimulus['BaselineGain'])

    else:
        theta1 = (stimulus['CenterPosition'] - stimulus['Modulation']['Width']/2) / VirtualTrackLength * np.pi * 2
        theta2 = (stimulus['CenterPosition'] + stimulus['Modulation']['Width']/2) / VirtualTrackLength * np.pi * 2
        visualization.add_zone(theta1, theta2, fillcolor=stimulus['Color'])

        SoundStimuliList[-1].initLocalizedSound(center=stimulus['CenterPosition'], 
                        width=stimulus['Modulation']['Width'], trackLength=VirtualTrackLength, 
                        maxGain=stimulus['BaselineGain'], minGain=stimulus['Modulation']['CutoffGain'])
        SoundStimuliList[-1].change_gain(-90.0) # start off turned off

    time.sleep(1.0)


from RewardZone import ClassicalRewardZone

RewardsList = []
for reward_name, reward in Config['RewardZones']['RewardZoneList'].items():
    if (reward['Type'] == 'Classical'):
        theta1 = reward['RewardZoneStart'] / VirtualTrackLength * np.pi * 2
        theta2 = reward['RewardZoneEnd'] / VirtualTrackLength * np.pi * 2
        visualization.add_zone(theta1, theta2, fillcolor=None, edgecolor=stimulus['Color'], 
                               hatch='....', width=1.33, alpha=1.0)
        RewardsList.append(ClassicalRewardZone((reward['RewardZoneStart'], reward['RewardZoneEnd']),
                reward['DispensePin'], reward['PumpRunTime'], reward['LickTimeout'],
                reward['MaxSequentialRewards'], (reward['ResetZoneStart'], reward['ResetZoneEnd'])) )
    else:
        raise(NotImplementedError("Reward types other than classical are not yet implemented"))




from SerialInterfaceSimulator import SerialInterface

Interface = SerialInterface()
## initiate encoder value ##
FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO = Interface.read_data()
initialUnwrappedencoder = UnwrappedEncoder 
print("initial unwrapped encoder value : ", UnwrappedEncoder)

## every 2 ms happens:
while(True):
    # last_ts = time.time()
    last_ts = time.monotonic()   # to match with miniscope timestamps (which is written in msec, here is sec)
    FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO = Interface.read_data()

    UnwrappedPosition = (UnwrappedEncoder - initialUnwrappedencoder) /Config['Maze']['EncoderGain'] * d  * np.pi 
    TrackPosition = UnwrappedPosition % VirtualTrackLength

    if (MasterTime % Config['Preferences']['HeartBeat']) == 0:
        print('Heartbeat {} - {} - 0x{:08b}'.format(MasterTime, UnwrappedPosition, GPIO[0]))

    for sound in SoundStimuliList:
        if(sound.localized):
            sound.pos_update_gain(TrackPosition)

    for reward in RewardsList:
        if reward.pos_reward(TrackPosition, MasterTime):
            print('Reward!')

    # Visualization
    #RingAngle = np.pi * 2 * (UnwrappedEncoder - initialUnwrappedencoder) / Config['Maze']['EncoderGain']
    RingAngle = (TrackPosition / VirtualTrackLength) * 2 * np.pi
    if (MasterTime % 100) == 0:
        visualization.move_mouse(RingAngle)

            
