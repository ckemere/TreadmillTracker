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

# JSON parameters: task settings
with open('defaults.yaml', 'r') as f:
    Config = yaml.safe_load(f)

print('Normalizing stimuli:')
StimuliList = Config['AuditoryStimuli']['StimuliList']
for stimulus_name, stimulus in StimuliList.items(): 
    print(' - ',stimulus_name)
    for key in Config['AuditoryStimuli']['Defaults']:
        if key not in stimulus:
            stimulus[key] = Config['AuditoryStimuli']['Defaults'][key]



#%%

#%%
#----------------------- parameters --------------
VirtualTrackDistance = Config['Maze']['Length'] #cm
d = Config['Maze']['WheelDiameter'] #cm diameter of the physical wheel; 150cm
count = 0


#%%
from RenderTrack import RenderTrack
track = RenderTrack(d/2)

for stimulus_name, stimulus in StimuliList.items():
    if stimulus['Type'] == 'Background':
        track.add_zone(0, 360, fillcolor=stimulus['Color'], width=0.5, alpha=0.75)
    else:
        theta1 = (stimulus['CenterPosition'] - stimulus['Modulation']['Width']/2) / VirtualTrackDistance * 360
        theta2 = (stimulus['CenterPosition'] + stimulus['Modulation']['Width']/2) / VirtualTrackDistance * 360
        track.add_zone(theta1, theta2, fillcolor=stimulus['Color'])


#%%
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

    if (MasterTime % 3000) == 0:
        print('Heartbeat {} '.format(MasterTime))

    LinearPosition = (UnwrappedEncoder - initialUnwrappedencoder) * d /4096 * np.pi 
    RingAngle = np.pi * 2 * (UnwrappedEncoder - initialUnwrappedencoder) / Config['Maze']['EncoderGain']
    
    if (MasterTime % 100) == 0:
        track.move_mouse(RingAngle)

            
