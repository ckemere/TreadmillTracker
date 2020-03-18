### Script to test multiple webcam synchronization ###
from Interface import Camera
import argparse
import time

# Camera parameters
parser = argparse.ArgumentParser(description='Test camera synchronization.')
parser.add_argument('-n', '--name', default='/home/kemerelab/Desktop/video',
                    help='Base filename for recordings.')
parser.add_argument('-c', '--count', type=int, default=2,
                   help='Number of camera devices.')
parser.add_argument('-f', '--fps', type=int, default=30,
                   help='Camera frame rate.')
parser.add_argument('-r', '--resolution', default='640x480',
                    help='Camera resolution (<width>x<height>).')
parser.add_argument('-t', '--time', type=float, default=10.0,
                    help='Duration of recording session in seconds.')
parser.add_argument('-a', '--audio', default='none', choices=['none', 'port', 'pulse'],
                    help='Camera audio API.')
args = parser.parse_args()

base_filename = args.name
num_cams = args.count
fps = args.fps
res = [int(d) for d in args.resolution.split('x')]
dt_total = args.time
audio = args.audio

# Instantiate camera objects
Cams = []
for i in range(num_cams):
    Cam = Camera(base_filename + str(i),
                 fps=fps,
                 resolution=res,
                 device='/dev/video{}'.format(2*i),
                 audio=audio)
    Cams.append(Cam)

# Start cameras successively
for Cam in Cams:
    print('Starting to record from {}...'.format(Cam.device))
    Cam.start()
t_start = time.time()

# Track elapsed time
dt_current = time.time() - t_start
frac = 0.1
while dt_current < dt_total:
    if (dt_current / dt_total) > frac:
        print('{:02.1f}%% complete...'.format(frac*100))
        frac += 0.1
    dt_current = time.time() - t_start

# Stop cameras
for Cam in Cams:
    Cam.stop()
t_stop = time.time()
print('Done.')

# Print elapsed time
dt = t_stop - t_start
hr = int(dt // 3600)
m = int((dt % 3600) // 60)
s = dt % 60
print('Recorded for {:02d}:{:02d}:{:05.2f}'.format(hr, m, s))