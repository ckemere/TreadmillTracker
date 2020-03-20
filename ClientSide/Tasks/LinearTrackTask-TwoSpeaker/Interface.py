import serial
import struct
from oscpy.client import OSCClient
import subprocess
import signal
import time

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

class AudioDevice:

    def __init__(self,
                 client_name,
                 port_name,
                 OSC_port,
                 device_type='output'):
    
    def add_sound(self, 
                  name, 
                  **kwargs):
        self.sounds[name] = Sounds(OscPort=self.osc_port,
                                   **kwargs)

    def play_sound(self, name):
        self.sounds[name].Play()

    def stop_sound(self, name):
        self.sounds[name].Stop()


class Camera:

    # See https://askubuntu.com/a/469125
    RECORD_CMD = ['killall', '--signal=USR1', 'guvcview']

    def __init__(self,
                 filename,
                 fps=30,
                 resolution=[640, 480],
                 cam_format='H264',
                 codec='raw',
                 audio='none'):
        """
        Creates an interface with a webcam via guvcview software.

        Args:
        - filename: Name (or filepath) of video file.
        - fps: Frame rate (Hz)
        - resolution: Video resolution to capture. 
            Only standard options are available.
        - cam_format: Camera output format. Must be FOURCC code 
            (i.e. str of len 4).
        - codec: Video codec to use. Note that this occurs *after* receiving the
            camera output. That is, guvcview will receive video data in 
            `cam_format` and then encode it by `codec` before writing to file. 
            This can slow frame rate as the buffer overflows from encoding time. 
            Options include: raw, mjpg, mpeg, flv1, wmv1, mpg2, mp43, dx50, h264, 
            vp80, theo
        - audio: Audio API to use. Options include: none, port, pulse
            (if enabled during build)
        """
        self.filename = filename
        self.fps = fps
        self.res = resolution
        self.cam_format = cam_format
        self.codec = codec
        self.audio = audio
        self.guvc_cmd = ['guvcview', 
                         '--fps={}'.format(self.fps),
                         '--resolution={}x{}'.format(self.res[0], self.res[1]),
                         '--format={}'.format(self.cam_format),
                         '--video_codec={}'.format(self.codec), 
                         '--video={}'.format(self.filename),
                         '--audio={}'.format(self.audio),
                         '--exit_on_term']
                         #'&>', '/dev/null'] # suppress annoying readouts

    def start(self):
        # Start guvcview with specified parameters
        self._subprocess = subprocess.Popen(self.guvc_cmd)
        print('Starting guvcview...')
        time.sleep(5.0) # allow time for guvcview to start

        # Send record signal to guvcview
        self._record()
    
    def _record(self):
        # Signal guvcview to start recording video
        subprocess.call(self.RECORD_CMD)

    def stop(self):
        # Send SIGINT to guvcview
        self._subprocess.send_signal(signal.SIGINT)

    def __enter__(self):
        # Begin recording video upon instantiation
        self.start()
        return self

    def __exit__(self, type, value, traceback):
        # Stop recording video upon exit
        self.stop()
        return True