#!/usr/bin/env python

import time
import argparse
import json
from subprocess import Popen, DEVNULL
import contextlib
from oscpy.client import OSCClient
import sys, atexit, signal

AUDIO_DIR = '../Audio/Stimuli/'
JACK_DEFAULTS = ['--realtime', 
                 '-P', '10',
                 '-d', 'alsa', 
                 '-p', '512', 
                 '-n' , '2', 
                 '-r', '96000',
                 '-d', 'hw:0,0']

# Command-line arguments
parser = argparse.ArgumentParser(description='Set up audio connections in JACK environment.')
parser.add_argument('-c', '--channel', type=int, 
                    help='Minimixer channel to alternate on/off.')
parser.add_argument('-i', '--input-device', type=str, 
                    help='Name of audio output device to play sound.')
parser.add_argument('-o', '--output-device', type=str, 
                    help='Name of audio output device to play sound.')
parser.add_argument('-s', '--side', type=str, default='right', choices=['left', 'right'],
                    help='output side of mixer (left or right)')
parser.add_argument('-p', '--param-file', default='defaults.json',
                    help='JSON file containing sound specifications')
parser.add_argument('-j', '--jack-options', type=str,
                    help='JACK daemon arguments as single string')
parser.add_argument('--output-dir', type=str, default='/home/kemerelab/Desktop',
                    help='Destination folder for recordings.')
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

with open('{}/jack_params.txt'.format(args.output_dir), 'w') as f:
    f.write(' '.join(jack_params))

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

def jackcapture_cmd(device):
    return ['/usr/local/bin/jack_capture',
            '--channels', '1',
            '--port', '{}:{}'.format(device['ClientName'], device['PortName']),
            '--timestamp',
            '--filename-prefix', '{}/{}'.format(args.output_dir, device['Name'])]

# Start audio connections within same context
with contextlib.ExitStack() as stack:
    # Start JACK daemon on sound card
    process = stack.enter_context(Popen(jack_cmd))
    atexit.register(process.send_signal, signal.SIGKILL) # send SIGKILL to terminate JACK daemon
    time.sleep(0.5)

    # Start minimixer for each output (speaker)
    cmds = [minimixer_cmd(s) for s in speaker_params]
    processes = [stack.enter_context(Popen(cmd, stdout=DEVNULL, stderr=None)) for cmd in cmds]
    time.sleep(0.5)

    # Send all audio files to all minimixers
    for s in speaker_params:
        cmds = [jackplay_cmd(s, 'in{}_{}'.format(p[0], args.side), p[1]) for p in sound_params]
        processes = [stack.enter_context(Popen(cmd, stdout=DEVNULL, stderr=DEVNULL)) for cmd in cmds]
        for process in processes:
            atexit.register(process.send_signal, signal.SIGKILL)
        print('Sending audio files {} to {}'.format(', '.join([p[1] for p in sound_params]), s['Name']))

    time.sleep(1.0)

    # Silence first channel of all minimixers (on by default)
    print('Silencing sounds...')
    for s in speaker_params:
        osc_client = OSCClient('127.0.0.1', int(s['OSCPort']))
        osc_client.send_message(b'/mixer/channel/set_gain',[1, -1000.0])
    time.sleep(1.0)

    # Select input and output device for testing
    for k, v in devices.items():
        if k.lower() == args.input_device.lower():
            device_in = v
            device_in['Name'] = k
        elif k.lower() == args.output_device.lower():
            device_out = v
            device_out['Name'] = k

    # Begin recording input/output streams
    cmds = [jackcapture_cmd(device_in), jackcapture_cmd(device_out)]
    processes = [stack.enter_context(Popen(cmd, stdout=DEVNULL, stderr=DEVNULL)) for cmd in cmds]
    for process in processes:
        atexit.register(process.send_signal, signal.SIGKILL)

    # Play test channel on and off
    for i in range(10):
        # Play sound
        print('Playing sound...')
        cmd = ['oscsend',
               'osc.udp://localhost:{}'.format(device_out['OSCPort']),
               '/mixer/channel/set_gain',
               'if', '{}'.format(args.channel), '0.0']
        process = Popen(cmd)
        process.wait()
        time.sleep(1.0)

        # Stop sound
        print('Stopping sound...')
        cmd = ['oscsend', 
               'osc.udp://localhost:{}'.format(device_out['OSCPort']),
               '/mixer/channel/set_gain',
               'if', '{}'.format(args.channel), '-1000.0']
        process = Popen(cmd)
        process.wait()
        time.sleep(1.0)

    print('Done.')

sys.exit()

