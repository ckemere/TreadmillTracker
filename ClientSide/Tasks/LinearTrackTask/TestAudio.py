#!/usr/bin/env python

#%%
import time


#%%
from oscpy.client import OSCClient
oscC = OSCClient('127.0.0.1', 12345)

from enum import Enum, auto, unique

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
from subprocess import Popen, DEVNULL

PinkNoiseStim = Sounds(SoundType.PinkNoise)
ToneCloudStim = Sounds(SoundType.ToneCloud)
ToneStim = Sounds(SoundType.Tone)

with Popen(['/usr/local/bin/jackminimix', '-a', '-p', '12345']) as p_jackminimix:
    time.sleep(1)
    print('Started jackminimix')
    with Popen(['/usr/local/bin/sndfile-jackplay', '-l0', 
            '-a=minimixer:in1_right', 'pink_noise.wav']) as p_pinknoise, \
        Popen(['/usr/local/bin/sndfile-jackplay', '-l0', 
            '-a=minimixer:in2_right', 'tone_cloud_gating.wav']) as p_tone_cloud:

        print('Getting ready to start')
        while True:
            time.sleep(5)
            print('Pink Noise')
            PinkNoiseStim.Play()
            ToneCloudStim.Stop()
            time.sleep(5)
            print('Tone cloud')
            PinkNoiseStim.Stop()
            ToneCloudStim.Play()

