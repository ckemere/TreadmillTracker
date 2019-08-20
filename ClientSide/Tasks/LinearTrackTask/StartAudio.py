#!/usr/bin/env python

import time
import argparse
import json
from subprocess import Popen, DEVNULL
import contextlib

AUDIO_DIR = '../../Audio/Stimuli/'

# Command-line arguments
parser = argparse.ArgumentParser(description='Run simple linear track experiment.')
parser.add_argument('-O', '--osc-port', type=int, default=12345, 
                   help='Serial port for OSC client')
parser.add_argument('-s', '--side', type=str, default='right', choices=['left', 'right'],
                    help='output side of mixer (left or right)')
parser.add_argument('-p', '--param-file', default='defaults.json',
                    help='JSON file containing sound specifications')
args = parser.parse_args()

# JSON parameters
with open(args.param_file) as f:
    d = json.loads(f.read())
    sounds = d['Sound'] # dict of dicts

sound_params = []
for k, v in sounds.items():
    sound_params.append([v['Channel'], v['Filename']])

jack_cmd = ['/usr/bin/jackd', '--realtime', '-P10','-d', 'alsa', '-p128', '-n2', '-r96000']
minimix_cmd  = ['/usr/local/bin/jackminimix', '-a', '-p', '12345']
def jackplay_cmd(channel, filename):
    return ['/usr/local/bin/sndfile-jackplay', '-l0', '-a=minimixer:{}'.format(channel), '{}{}'.format(AUDIO_DIR, filename)]

with Popen(jack_cmd) as p_jack:
    time.sleep(0.5)
    with Popen(minimix_cmd, stdout=DEVNULL) as p_jackminimix:
        time.sleep(0.25)
        with contextlib.ExitStack() as stack:
            cmds = [jackplay_cmd('in{}_{}'.format(p[0], args.side), p[1]) for p in sound_params]
            processes = [stack.enter_context(Popen(cmd, stdout=DEVNULL, stderr=DEVNULL)) for cmd in cmds]
            print('Playing audio files {}'.format(', '.join([p[1] for p in sound_params])))
            while True:
                time.sleep(10)
                print('Audio alive')

