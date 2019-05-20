

## Basic Plan: JACK audio with a mixer controlled by OSC commands

### Testing

#### Install ubuntu studio control
install - ubuntustudio-controls jack-tools ubuntustudio-performance-tweaks

#### jackminimix
liblo-tools liblo-dev libjack-jackd2-dev autotools-dev, autoconf


#### jackmeter
Useful to see stuff working


#### How do we test

  - Use `mplayer -ao jack Linchirp.ogg -loop 0` to loop audio files (plays wav also).
  - Start up JackMiniMixer. Start as `./jackminimix -v` (verbose option is useful for feedback.)
  - Use qjackctl to make sure that everything gets connected (i.e., different audio sources to
    different mixer channels) graphically. There probably is a command line way too?
  - Send OSC commands to control stuff. E.g., `oscsend "osc.udp://localhost:14161" "/mixer/channel/set_gain" if 2 -10.0`
  - *NOTE* If you don't hear anything or it's too loud, check alsamixer for volume with JACK
    running


### OSC commands

#### Python
TBD

#### Command line

example `oscsend "osc.udp://localhost:14161" "/mixer/channel/set_gain" if 2 -10.0`
