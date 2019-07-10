

## Basic Plan: JACK audio with a mixer controlled by OSC commands

This has been tested on an updated Ubuntu 18.04 installation on the desktop and on a Ubuntu 19.04 server installation on the Raspberry Pi (i.e., ubuntu-19.04-preinstalled-server-armhf+raspi3.img.xz from here:[http://cdimage.ubuntu.com/releases/19.04/release/]).

### Testing

***Remember:*** If stuff is weird on the laptop it may be because anaconda has
stolen our path!!!

#### Install jack
`sudo apt install libjack-jackd2-dev jack-tools`

# (optional) Install ubuntu studio control
ubuntustudio-controls ubuntustudio-performance-tweaks

#### jackminimix
[https://github.com/njh/jackminimix/]

liblo-tools liblo-dev autotools-dev autoconf automake

Note that there's a 'GAIN_FADE_RATE` value which is defined as 400 db/minute.
This gives nice transitions, but extends sounds a bit.

#### jackmeter
Useful to see stuff working

#### sndfile-jackplay
[https://github.com/erikd/sndfile-tools/]

libtool libsndfile1-dev libsamplerate0-dev libfftw3-dev libcairo2-dev

#### GPIO

To install the newest and greatest RPi.GPIO, we have to `pip` from the source
repository. Since it's hosted in SVN, we need to `apt install mercurial`. Then,
we can `pip3 install hg+http://hg.code.sf.net/p/raspberry-gpio-python/code#egg=RPi.GPIO`
This solves issues with `/proc/cpuinfo`, but still raises a `Cannot access
/dev/mem` error unless run with root, and even then you need a fancy thing in
the `cmdline.txt` file. See [README.md].


#### How do we test

  - Use `mplayer -ao jack Linchirp.ogg -loop 0` to loop audio files (plays wav also).
  - Start up JackMiniMixer. Start as `./jackminimix -v` (verbose option is useful for feedback.)
  - Use qjackctl to make sure that everything gets connected (i.e., different audio sources to
    different mixer channels) graphically. There probably is a command line way too?
  - Send OSC commands to control stuff. E.g., `oscsend "osc.udp://localhost:14161" "/mixer/channel/set_gain" if 2 -10.0`
  - *NOTE* If you don't hear anything or it's too loud, check alsamixer for volume with JACK
    running


  - We've added a gapless looping wave file player from the sndfile-tools
    library. For some reason the packaged version doesn't connect to the jack
    daemon, but we've copied the source into the PlaySound directory. Compile
    with ' '
  - To play, use `./PlaySound/jplay -l0 -a=minimixer:in1_right 15khz_sine.wav`.
    Note that the `-l 0` flag loops infinitely (gapless FTW!) and the `-a` flag
    specifies to send data to the minimixer. Can just skip that to test.
    Other stuff is here -
    [http://mega-nerd.com/libsndfile/tools/](http://mega-nerd.com/libsndfile/tools/).

  - `jack_lsp --connections` is a useful command for seeing what the connections are.

  - The $1M question is what is the latency from detecting an event (i.e., a
    nose poke) to modulating the sound. To test this, we need to run the JACK
    daemon, the minixer, and a Python script which sends osc commands based on
    IO.
       1. Start JACK: `jackd -R -P10 -v -d alsa -p64 -n2 -P hw:1,0` (`-R` is
          realtime, `-P` priority is 10, -v verbose, `-d` set alsa output device,
          `-p 64` set the buffer to be small and `-n 2` have a minimal number of
          buffers, and `-P hw:1,0` set the proper soundcard) We could also add
          a `-r 192000` to set the sampling rate.

       2. Start the minimixer: `jackminimix -a -p 12345` (optionally `-v` to
	  see commands coming through.) Note that `12345` is the port for osc
          control. You can change it to something else, but make sure your other
          code matches.

       3. Start an audio loop playing into the mixer: `./PlaySound/jplay -l0
          -a=minimixer:in1_right 15khz_sine.wav`

       4. Figure out what the minimix port is, and then set it and run the test
          script `sudo python3 TestPiAudioControl.py`. It has to be run as root
          because RPi.GPIO appears to require it. This script toggles pin 17
          every 200 ms, while continuously reading from the UART data stream. If
          it detects a changed value on pin 16, it sends an OSC message to
          either make the gain 0.0 dB (full volume) or -30.0 dB. Putting the
          oscilloscope on pin 16 and the speaker, we see sound changes starting
          about 6-7 ms following the edge of the pin. This is consistent with
          first a worst-case UART sampling delay of 2 ms  followed by a few
          sound buffers worth of latency. (Note that with `-p64`, each buffer is
          *1.3 ms* long, so with `n2`, that means a minimum of 2.6 ms of
          latency. I guess the minimixer 

### OSC commands

#### Python

  - Wrote a Python script to make a pure tone sinewave at arbitrary sampling
    rates - `GenerateTone.py`. Note that the gain was weird when we tried to
    write it as a mono file instead of stereo.

  - Python script that sends OSC commands uses `oscpy` library. `pip3 install
    oscpy` gets it.

#### Command line

example `oscsend "osc.udp://localhost:12345" "/mixer/channel/set_gain" if 2 -10.0`
