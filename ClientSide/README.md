# Notes on using the Treadmill Tracker

### Using the direct USB interface

#### Expectations
We are sending 9 byte messages at 256000 baud. Since there is an extra stop byte per serial
transaction, our messages are actually 9 x 9 or 81 bits long, and so our sustained rate can at
most be 256000/81, or more interestingly, our minimum transaction period can be 81/256000 or
0.31 ms. Given that the USB transaction rate is expected to have a period of at least 1 ms, we
suspect the best we can do is 2 ms data transmission rates.

#### Quantifying latency
There's several ways of quantifying latency. 

##### Message rate latency
Since in the absence of any triggered outputs we know the rate that the MSP430 is sending data,
for this project we can also ask what the time difference between receipt of serial packets is.
`TestInterEventInterval.py` records the time of each packet to a data file. When we do this
with a naive Ubuntu 18.04 installation, we find that we can't get packets more than about every
15 ms. Given that the USB 2.0 interval should be 0.125 ms or somesuch, this is disappointing.
Reading through the driver source
(https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/usb/serial/ftdi_sio.c),
we find reference to a "latency timer". Reading up on that, it appears that this is what
controls how often the FTDI chip sends data (either whenever its 512 byte buffer is full *or*
when the latency timer expires). More reading tells us that we can find the current value of
the latency timer by looking in /sys/bus/usb-serial/devices/ttyUSB0/latency_timer (or whatever
the proper ttyUSB is). Reading this file, we find the value "16"!? If we try to write to this
file, it gives an error message (even with superuser). A little more searching, and we find a
comment on a [blog post](https://projectgus.com/2011/10/notes-on-ftdi-latency-with-arduino/)
that points out we can make a UDEV rule. So, finally, we edit the file
`/etc/udev/rules.d/60-ftdi-latency.rules`:

```
# Lower the latency timer for the FTDI serial interface
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

Now, setting the transmit rate to 5 ms, we find a median rate of 5 ms, with a max of 6.3 ms
measured over 27K messages! With a transmit rate of 2 ms, we find a median rate of 2 ms, with a
max of 3.2 over 50K messages.

An interesting and open question is how to set the latency timer for Windows or OSX.

##### Closed loop latency
Another way of measuring latency is to ask how fast we can see a change that we trigger.
`TestClosedLoop.py` uses the reported time stamps to do this. The basic idea is to wait until a
particular timestamp is received, trigger an output, and then see what the timestamp is for the
trigger-based message comes back. When the latency timer was large, this gave interesting
results. Now, it mainly reports 1 ms results, which aren't particularly interesting.


### Using the Raspberry Pi interface
When plugged into the Raspberry pi, the serial device is `/dev/ttyAMA0`, which is different from the default for the FT230 driver, which is `/dev/ttyUSB0`!

To control GPIO with the Raspberry Pi, there are a number of libraries. Our favorite,
`pigpio` is not yet working with a 64-bit kernel. The new kernel library for IO
is `libgpiod`. Hopefully we'll eventually switch to that interface. For now,
we're using RPi.GPIO, which is a fairly standard Raspberry PI Python GPIO
library. RPi.GPIO is also sensitive to architecture. See
[here](https://alioth-lists.debian.net/pipermail/pkg-raspi-maintainers/Week-of-Mon-20190318/000333.html)
for a description of what needs to be done to get it to work. Basically two
things: (1) making sure that the CPU detection code doesn't rely on
`/proc/cpuinfo` (this is already patched in RPi.GPIO, but a recent version needs
to be used) and (2) making sure that `/dev/mem` can be accessed. As described in
the link above, 
> So, it fails to open /dev/mem. That's because the Debian kernel is
> built with CONFIG_DEVMEM_STRICT=y. To work around that, we need to add
> iomem=relaxed to the kernel command line in /boot/firmware/cmdline.txt

