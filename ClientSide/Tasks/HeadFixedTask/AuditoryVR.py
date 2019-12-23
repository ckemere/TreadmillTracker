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
import datetime
import os
import argparse
import yaml
import csv
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

now = datetime.datetime.now()
log_filename = '{}{}.txt'.format('Log', now.strftime("%Y-%m-%d %H%M"))
log_filename = os.path.join(args.output_dir, log_filename)


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
TrackTransform = None

if Config['Maze']['Type'] == 'VR':
    VirtualTrackLength = Config['Maze']['Length'] #cm
    d = Config['Maze']['WheelDiameter'] #cm diameter of the physical wheel; 150cm
count = 0


#%%
from RenderTrack import RenderTrack

visualization = RenderTrack(track_length=VirtualTrackLength)

from SoundStimulus import SoundStimulus

SoundStimuliList = []

for stimulus_name, stimulus in StimuliList.items():

    filename = stimulus['Filename']
    if Config['Preferences']['AudioFileDirectory']:
        filename = os.path.join(Config['Preferences']['AudioFileDirectory'], filename)
    print('Loading: {}'.format(filename))
    SoundStimuliList.append(SoundStimulus(filename=filename))

    if stimulus['Type'] == 'Background':
        visualization.add_zone_position(0, VirtualTrackLength, fillcolor=stimulus['Color'], width=0.5, alpha=0.75)

        SoundStimuliList[-1].change_gain(stimulus['BaselineGain'])

    else:
        visualization.add_zone_position(stimulus['CenterPosition'] - stimulus['Modulation']['Width']/2, 
                               stimulus['CenterPosition'] + stimulus['Modulation']['Width']/2, 
                               fillcolor=stimulus['Color'])

        SoundStimuliList[-1].initLocalizedSound(center=stimulus['CenterPosition'], 
                        width=stimulus['Modulation']['Width'], trackLength=VirtualTrackLength, 
                        maxGain=stimulus['BaselineGain'], minGain=stimulus['Modulation']['CutoffGain'])
        SoundStimuliList[-1].change_gain(-90.0) # start off turned off

    time.sleep(1.0)


from RewardZone import ClassicalRewardZone, OperantRewardZone

RewardsList = []
for reward_name, reward in Config['RewardZones']['RewardZoneList'].items():
    if (reward['Type'] == 'Classical') | (reward['Type'] == 'Operant'):
        visualization.add_zone_position(reward['RewardZoneStart'], reward['RewardZoneEnd'], 
                        fillcolor=None, edgecolor=stimulus['Color'], hatch='....', width=1.33, alpha=1.0)

        if (reward['Type'] == 'Classical'):
            RewardsList.append(ClassicalRewardZone((reward['RewardZoneStart'], reward['RewardZoneEnd']),
                    reward['DispensePin'], reward['PumpRunTime'], reward['LickTimeout'],
                    reward['MaxSequentialRewards'], (reward['ResetZoneStart'], reward['ResetZoneEnd'])) )

        elif (reward['Type'] == 'Operant'):
            RewardsList.append(OperantRewardZone((reward['RewardZoneStart'], reward['RewardZoneEnd']),
                    reward['LickPin'], reward['DispensePin'], reward['PumpRunTime'], reward['LickTimeout'],
                    reward['MaxSequentialRewards'], 
                    (reward['ResetZoneStart'], reward['ResetZoneEnd'])) )            
    else:
        raise(NotImplementedError("Reward types other than classical are not yet implemented"))



from SerialInterfaceSimulator import SerialInterface

Interface = SerialInterface()
## initiate encoder value ##
FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO = Interface.read_data()
initialUnwrappedencoder = UnwrappedEncoder 
print("initial unwrapped encoder value : ", UnwrappedEncoder)

with open(log_filename, 'w', newline='') as log_file:
    writer = csv.writer(log_file)

    while(True):
        ## every 2 ms happens:
        last_ts = time.monotonic()   # to match with miniscope timestamps (which is written in msec, here is sec)
        FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO = Interface.read_data()

        writer.writerow([MasterTime, GPIO, Encoder, UnwrappedEncoder, last_ts])

        if Config['Maze']['Type'] == 'VR':
            UnwrappedPosition = (UnwrappedEncoder - initialUnwrappedencoder) / Config['Maze']['EncoderGain'] *d *np.pi 
            TrackPosition = UnwrappedPosition % VirtualTrackLength
        else:
            # GPIO to Position transformation
            # Use this for controlling behavior on a physical track where position is not tracked by a rotary encoder
            # TrackPosition = TrackTransform.convert(GPIO[0])
            TrackPosition = 0 

        if (MasterTime % Config['Preferences']['HeartBeat']) == 0:
            print('Heartbeat {} - {} - 0x{:08b}'.format(MasterTime, TrackPosition, GPIO[0]))


        # Stimulus
        for sound in SoundStimuliList:
            if(sound.localized):
                sound.pos_update_gain(TrackPosition)

        # Reward
        for reward in RewardsList:
            if reward.pos_reward(TrackPosition, GPIO[0], MasterTime):
                print('Reward!')

        # Visualization
        if (MasterTime % 100) == 0:
            visualization.move_mouse_position(TrackPosition)

            
