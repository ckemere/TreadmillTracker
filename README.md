# Raspberry pi based interface for a rodent treadmill

  - MSP430 based HW data acquisition "hat"
  - Rotational angle tracking using the CUI AMT 102/3 quadrature encoders
  - 1 kHz master clock for timestamping 8 IO lines

## Setup overview

### PiHat
The custom Raspberry Pi hat contains several useful features.
- **Eight GPIO pins** can be used for either receiving or sending signals.
- **Encoder data (AMT102)** is received through the six pins at the top of the board.  
- **2 kHz master clock (DSC6083)** reliably timestamps input data.
- **Button** ?
- **MSP430 microcontroller** packages and sends serial messages containing timestamped data every 2 ms. Programmed via seven pins at bottom of board.
- **Shield** interfaces with Raspberry Pi pins, including the eight GPIO pins connected to RaspPi GPIO pins.

### Accessories
#### Rotary encoder
The AMT102 sends signals in increments with resolution of 4096 steps/revolution. Thus each pulse signals that the wheel has turned 1/4096 of a revolution. For more details, look at the specs in the data sheet contained in *Firmware*.

#### Lick sensor
The lick sensor sends digital signals to one of the Hat's GPIO pins to indicate a licking event is occurring. More details in lick-sensor repository.

#### Nose-poke sensor
The nose-poke sensor sends digital signals to one of the Hat's GPIO pins to indicate a nose-poke event is occurring.

#### Syringe pump
The syringe pump receives 5V pulses from a GPIO pin on the Hat to dispense liquid rewards. The duration of the pulse determines the volume of reward dispensed (given a syringe diameter and pump speed).

#### Sound card
The sound card converts the digital audio file (.wav) to an analog signal to drive the external speakers. It can either be integrated with the host computer (if using a desktop), or as a shield (if using a Raspberry Pi; the HiFiBerry Amp2, for example, uses built-in drivers in the Raspberry Pi Linux kernel to obtain high-quality I2C digital audio from the GPIO pins, which it converts to an analog signal for speaker input. See [here](https://www.hifiberry.com/build/documentation/configuring-linux-3-18-x/) for more details.)

#### Breakout board
In order to streamline I/O connections between the custom hat and accessories, a custom breakout board shield connects GPIO pins to the rings of 3.5 mm audio jacks. Additionally, the hat Vdd (3.6 V) can be routed to supply voltage to the tips of the audio jacks. With this setup, audio cables can transmit GPIO signal to input and from output devices listed above. For example, audio cables can carry lick and nose-poke sensor digital signals, or send dispense signals to the syringe pumps.

### Host computer
The host computer can either be a Raspberry Pi or a desktop computer. If using a Raspberry Pi, then the custom hat and sound card shield can be stacked on top of one another to form a three-layer, stand-alone device. Otherwise, the host computer is connected to the custom hat via USB, and the native desktop sound card is used to drive the external speakers. In either case, the following components must be installed on the computer.

#### Mixer
The combination of the JACK Audio Connection Kit and minimixer allows for cross-talk between audio sources to function as a software mixer for different audio signals. The JACK daemon runs in the background as a sound server, built on top of the Advanced Linux Sound Architecture (ALSA), for a particular output device (sound card). In a separate process, the minimixer receives different audio feeds in its virtual channels, and controls each sound level of each channel that it feeds into the JACK daemon based on OSC commands it receives. Thus the user has control to dynamically change the output levels of different virtual channels in the minimixer.

#### OSC interface
The mixer, in turn, is interfaced through an OSC package, such `oscpy` in Python, that binds and sends OSC commands to the same port as the mixer. The commands direct the mixer to increase or decrease the gain for each channel in the mixer.

#### Control script
The control script must manage I/O from all the above components, including:
- Processing **behavior input** from the custom hat to determine the current behavior state. The input messages via the serial interface include current time, wheel encoder position, and GPIO values.
- Controlling **audio output** by launching appropriate audio subprocesses (e.g. daemon, mixer) and sending OSC commands to the mixer based on the current behavior state.
- Controlling **behavior output** by sending dispense pulses to the GPIOs associated with syringe pumps based on the current behavior state.
