import argparse
import struct
import time
from Interface import SerialInterface

# IDs of GPIO pins on PCB
GPIO_IDs = [16, 17, 22, 23, 24, 25, 26, 27]

# Command line args
parser = argparse.ArgumentParser(description='Run syringe pump(s) to prime wells.')
parser.add_argument('wells', type=int, nargs='*',
                    help='GPIO pins of wells to prime.')
parser.add_argument('-P', '--serial-port', default='/dev/ttyS0',
                   help='TTY device for USB-serial interface (e.g., /dev/ttyUSB0 or COM10)')
parser.add_argument('-t', '--time', type=float, default=1.0,
                    help='duration of each pulse (seconds)')
parser.add_argument('-n', '--num-pulses', type=int, default=1,
                    help='number of pulses to send to syringe pump')
args = parser.parse_args()

# Create bitmask for wells
DispenseMask = 0
for well in args.wells:
    DispenseMask += 2**(GPIO_IDs.index(well))
DispenseMask = struct.pack('<B', DispenseMask)

# Create serial interface with PCB
Interface = SerialInterface(SerialPort=args.serial_port)

# Send pulses to syringe pumps
for n in range(args.num_pulses):
    print('Sending pulse %d of %d...' % (n+1, args.num_pulses))
    Interface.send_byte(DispenseMask)
    time.sleep(args.time)
    Interface.send_byte(b'\x00') # reset wells
    time.sleep(args.time)
print('Done.')