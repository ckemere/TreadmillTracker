import pigpio

pi = pigpio.pi()

pi.write(16,1)

pi.stop()
