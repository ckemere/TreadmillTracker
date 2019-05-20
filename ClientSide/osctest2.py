
#%%
# https://github.com/kivy/oscpy

from oscpy.server import OSCThreadServer
from time import sleep
from oscpy.client import OSCClient


def callback(values):
    print("got values: {}".format(values))

#%%
oscS = OSCThreadServer(default_handler=callback)
sock = oscS.listen(address='0.0.0.0', port=12345, default=True)

#%%
oscS.bind(b'/address', callback)

#%%
oscS.send_message(b'/ping',[],'127.0.0.1',14161)

sleep(5)
#%%

sleep(5)
oscS.stop()

#%%
