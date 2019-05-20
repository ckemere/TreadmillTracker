
#%%
# Import needed modules from osc4py3
from osc4py3.as_eventloop import *
from osc4py3 import oscbuildparse

#%%
def handlerfunction(s, x, y):
    # Will receive message data unpacked in s, x, y
    print(s)
    print(x)
    print(y)
    print('Ran handler')
    pass

#%%
# Start the system.
osc_startup()

# Make server channels to receive packets.
osc_udp_server("127.0.0.1", 14484, "somethingelse")

#%%
# Associate Python functions with message address patterns, using default
# argument scheme OSCARG_DATAUNPACK.
osc_method("*", handlerfunction)
# Too, but request the message address pattern before in argscheme
#osc_method("/test/*", handlerfunction2, argscheme=osm.OSCARG_ADDRESS + 
#    osm.OSCARG_DATAUNPACK)

#%%
osc_udp_client("127.0.0.1", 12887, "somethingelse2")


#%%
import time
# Periodically call osc4py3 processing method in your event loop.
finished = False
while not finished:
    msg = oscbuildparse.OSCMessage("/ping","",None)
    osc_send(msg, "somethingelse2")
    # …
    osc_process()
    print('sleeping')
    time.sleep(1)
    # …
#%%
# Properly close the system.
osc_terminate()

#%%
from oscpy.server import OSCThreadServer
from time import sleep
from oscpy.client import OSCClient


def callback(values):
    print("got values: {}".format(values))

#%%
oscS = OSCThreadServer()
sock = oscS.listen(address='0.0.0.0', port=12345, default=True)

#%%
oscC = OSCClient('127.0.0.1', 12887)


oscS.bind(b'/address', callback)
oscC.bind(b'/address', callback)

oscC.send_message(b'/ping',[])

sleep(1000)
oscS.stop()

#%%
