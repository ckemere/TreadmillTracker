import serial
import struct
from oscpy.client import OSCClient

class SerialInterface():
    def __init__(self, SerialPort='/dev/ttyS0'):
        print('Connecting to serial/USB interface {} and synchronizing.'.format(SerialPort))
        self.serial = serial.Serial(port=SerialPort,
            baudrate = 256000,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.2
        )

        # Synchronize immediately after opening port!

        # We read in a large buffer of data and find which offset is the start of the packets
        K = 3 # This code works for 100 but not 1000. Maybe related to buffer size???
        self.MessageLen = 14
        x=self.serial.read(self.MessageLen*(K+1))
        assert(len(x) == self.MessageLen*(K+1))
        # print(x) # useful for debugging....
        # Find offset in this set
        index = 0
        while(True):
            print(x[index:])
            continueFlag = False
            for k in range(K):
                if ( x.index(b'E',index + k*self.MessageLen) - (k*self.MessageLen + index)) != 0:
                    continueFlag = True
            if continueFlag:
                index = index + 1
            else:
                break
            if (index > (self.MessageLen-1)):
                print('Reached end with bad index')
                assert(False)
                break

        print('Found index: {}'.format(index))

        x = self.serial.read(index) # read the last little bit of the bad block, and we are in sync!


    def read_data(self):
        x=self.serial.read(self.MessageLen)
        assert(len(x)==self.MessageLen)
        FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO  = struct.unpack('<cBLhlBx', x)
        # print('Flag: {}. Clocks: {}. Encoder: {}. Unwrapped: {}, GPIO: 0x{:08b}'.format( 
            # FlagChar, MasterTime, Encoder, UnwrappedEncoder, GPIO))
        assert(FlagChar == b'E')
        return FlagChar, StructSize, MasterTime, Encoder, UnwrappedEncoder, GPIO

    def send_byte(self,data):
        if data is not None:
            self.serial.write(data)

class Sounds:
    def __init__(self, 
                 Name=None,
                 Channel=0,
                 OnVolume=0.0, 
                 OffVolume=-1000.0,
                 OscPort=None):
        self.channel = Channel
        if Name is not None:
            self.name = Name
        else:
            self.name = 'channel {}'.format(Channel)
        self.OnVolume = float(OnVolume)
        self.OffVolume = float(OffVolume)
        if OscPort is not None:
            self.oscC = OSCClient('127.0.0.1', int(OscPort))
        else:
            self.oscC = OSCClient('127.0.0.1', 12345)

    def Play(self):
        if self.channel != 0:
            self.oscC.send_message(b'/mixer/channel/set_gain',[int(self.channel), self.OnVolume])
            print('Playing', self.name)

    def Stop(self):
        if self.channel != 0:
            self.oscC.send_message(b'/mixer/channel/set_gain',[int(self.channel), self.OffVolume])
            print('Stopping', self.name)