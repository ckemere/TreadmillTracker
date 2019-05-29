import math
import wave
import struct

nchannels = 2
sampwidth = 2
framerate = 48000
nframes = framerate/100
comptype = "NONE"
compname = "not compressed"
amplitude = 32767.0/200
frequency = 18000

wav_file = wave.open('15khz_sine.wav', 'w')
wav_file.setparams((nchannels, sampwidth, framerate, nframes, comptype, compname))
for i in range(nframes):
    sample = math.sin(2*math.pi*frequency*(float(i)/framerate))*amplitude
    wav_file.writeframesraw(struct.pack('<hh', int(sample), int(sample)))
wav_file.close()

