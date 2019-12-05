
#import serial

from itertools import cycle
import csv
import time

class SerialInterface():

    def __init__(self, SerialPort='/dev/ttyS0'):
        self.GPIO_state = b'\x00' # initialize GPIO state to 0

        self.EncoderData = []
        self.MasterTime = 0 # supposed to be a 4 byte int

        # Load a sample data file
        print('Loading sample data file for simulation.')
        with open('shorterlog.txt', newline='') as csvfile: 
            reader = csv.reader(csvfile, delimiter=',') 
            for row in reader: 
                self.EncoderData.append(int(row[2])) # Append the (wrapped) Encoder data

        self.UnwrappedEncoder = self.EncoderData[0] # supposed to be an unsigned 4 byte int
        self.Encoder = self.EncoderData[0] # supposed to be a 2 byte int
        self.EncoderGenerator = cycle(self.EncoderData) # make it circular
        self.Clock = time.monotonic()

        
    def read_data(self):
        FlagChar = b'E'
        StructSize = 14
        self.MasterTime = self.MasterTime + 2 # 2 ms ticks
        NextEncoder = next(self.EncoderGenerator)
        DiffEncoder = NextEncoder - self.Encoder
        self.UnwrappedEncoder += DiffEncoder
        self.Encoder = NextEncoder
        #FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO  = struct.unpack('<cBLhlBx', x)
        # c = char, B = unsigned char, L = unsigned long (4 bytes), h = short (2 bytes), l = long, x = pad byte
        assert(FlagChar == b'E')

        currentClock = time.monotonic()
        while (currentClock - self.Clock) < 0.002:
            time.sleep((0.002 - (currentClock-self.Clock))*0.8)
            currentClock = time.monotonic()
        self.Clock = currentClock 
        return FlagChar, StructSize, self.MasterTime, self.Encoder, self.UnwrappedEncoder, self.GPIO_state

    def send_byte(self,data):
        if data is not None:
            if isinstance(data, bytes) and len(data) == 1:
                GPIO_state = data
            else:
                raise(ValueError('data was not a length 1 byte.'))