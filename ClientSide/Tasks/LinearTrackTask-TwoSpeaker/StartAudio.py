#!/usr/bin/env python

import time
import argparse
import json
from subprocess import Popen, DEVNULL
import contextlib

AUDIO_DIR = '../../Audio/Stimuli/'
JACK_DEFAULTS = ['--realtime', 
                 '-P', '10',
                 '-d', 'alsa', 
                 '-p', '512', 
                 '-n' , '2', 
                 '-r', '96000',
                 '-d', 'hw:0,0']

# Command-line arguments
parser = argparse.ArgumentParser(description='Set up audio connections in JACK environment.')
parser.add_argument('-s', '--side', type=str, default='right', choices=['left', 'right'],
                    help='output side of mixer (left or right)')
parser.add_argument('-p', '--param-file', default='defaults.json',
                    help='JSON file containing sound specifications')
parser.add_argument('-j', '--jack-options', type=str,
                    help='JACK daemon arguments as single string')
args = parser.parse_args()

# JSON parameters
with open(args.param_file) as f:
    d = json.loads(f.read())
    sounds = d['Sound'] # dict of dicts
    devices = d['AudioIO'] # dict of dicts

# Configure audio settings
sound_params = []
for k, v in sounds.items():
    sound_params.append([v['Channel'], v['Filename']])

speaker_params = []
for k, v in devices.items():
    if v['Type'].lower() == 'output':
        v['Name'] = k
        speaker_params.append(v)

jack_params = []
if len(args.jack_options) == 0:
    jack_params = JACK_DEFAULTS
else:
    jack_params = args.jack_options.split(' ')

# Configure JACK, minimixer, and jackplay commands
jack_cmd = ['/usr/bin/jackd'] + jack_params

def minimixer_cmd(speaker):
    return ['/usr/local/bin/jackminimix', 
            '-{}'.format(args.side[0]), '{}:{}'.format(speaker['ClientName'], speaker['PortName']), 
            '-p', '{}'.format(speaker['OSCPort']),
            '-n', 'minimixer-{}'.format(speaker['Name'])]

def jackplay_cmd(speaker, channel, filename):
    return ['/usr/local/bin/sndfile-jackplay', 
            '-l', '0', 
            '-a=minimixer-{}:{}'.format(speaker['Name'], channel), 
            '{}{}'.format(AUDIO_DIR, filename)]

# Start audio connections within same context
with contextlib.ExitStack() as stack:
    # Start JACK daemon on sound card
    _ = stack.enter_context(Popen(jack_cmd))
    time.sleep(0.5)

    # Start minimixer for each output (speaker)
    cmds = [minimixer_cmd(s) for s in speaker_params]
    processes = [stack.enter_context(Popen(cmd, stdout=DEVNULL, stderr=DEVNULL)) for cmd in cmds]
    time.sleep(0.5)

    # Send all audio files to all minimixers
    for s in speaker_params:
        cmds = [jackplay_cmd(s, 'in{}_{}'.format(p[0], args.side), p[1]) for p in sound_params]
        processes = [stack.enter_context(Popen(cmd, stdout=DEVNULL, stderr=DEVNULL)) for cmd in cmds]
        print('Sending audio files {} to {}'.format(', '.join([p[1] for p in sound_params]), s['Name']))

    # Run continuously
    while True:
        time.sleep(10)
        print('Audio alive')

