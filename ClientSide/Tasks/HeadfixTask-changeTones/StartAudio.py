#!/usr/bin/env python

#%%
import time
import argparse
from subprocess import Popen, DEVNULL

AUDIO_DIR = '../../Audio/Stimuli/'

### Maybe should add argcomplete for this program?

parser = argparse.ArgumentParser(description='Run simple linear track experiment.')
parser.add_argument('-O', '--osc-port', type=int, default=12345, 
                   help='Serial port for OSC client')

args = parser.parse_args()


jack_cmd = ['/usr/bin/jackd', '--realtime', '-P10','-d', 'alsa', '-p128', '-n2', '-r96000']
minimix_cmd  = ['/usr/local/bin/jackminimix', '-a', '-p', '12345']
def jackplay_cmd(channel, filename):
    return ['/usr/local/bin/sndfile-jackplay', '-l0', '-a=minimixer:{}'.format(channel), '{}{}'.format(AUDIO_DIR, filename)]

with Popen(jack_cmd) as p_jack:
    time.sleep(0.5)
    with Popen(minimix_cmd, stdout=DEVNULL) as p_jackminimix:
        time.sleep(0.25)
        with Popen(jackplay_cmd('in1_right', 'pink_noise.wav'), stdout=DEVNULL, stderr=DEVNULL) as p_pinknoise, \
            Popen(jackplay_cmd('in2_right', 'tone_3kHz.wav'), stdout=DEVNULL, stderr=DEVNULL) as p_toned1, \
            Popen(jackplay_cmd('in3_right', 'tone_6kHz.wav'), stdout=DEVNULL, stderr=DEVNULL) as p_toned2, \
            Popen(jackplay_cmd('in4_right', 'tone_12kHz.wav'), stdout=DEVNULL, stderr=DEVNULL) as p_toned3:
            # Popen(jackplay_cmd('in2_right', 'tone_cloud_gating.wav'), stdout=DEVNULL, stderr=DEVNULL) as p_tone_cloud, \
                print('Started')
                while True:
                    time.sleep(10)
                    print('Audio alive')

