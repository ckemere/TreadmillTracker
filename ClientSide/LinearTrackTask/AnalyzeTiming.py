#%%
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
#import mpld3

#%%
df = pd.read_csv('Log2019-07-03 0051.txt', names=['MasterTime','GPIO','SysTS'])

#%%
TimeStamps = df['MasterTime']
SysTS = df['SysTS']

diffTS = np.diff(SysTS)

#%%
sns.distplot(diffTS)

#%%
print(np.min(diffTS))
print(np.max(diffTS))


#%%
mpld3.enable_notebook()
fig,ax = plt.subplots(1,1,figsize=(12,6))
ax.plot(TimeStamps, EncoderData)

boundaries = np.ones_like(TimeStamps) * -4095
ax.plot(TimeStamps, boundaries, 'r--')

ax.set_ylim(-4110, -4000,)

#%%
Speed = np.diff(EncoderData)
Speed1 = Speed[0::2]
Speed2 = Speed[1::2]

#%%
Speed1b = Speed1[(Speed1 != 0)&(np.abs(Speed1) < 200)]
Speed2b = Speed2[(Speed2 != 0)&(np.abs(Speed2) < 200)]
sns.distplot(Speed1b,100)
sns.distplot(Speed2b,100)

#%%
fig,ax = plt.subplots(1,1,figsize=(12,12))
ax.plot(Speed1, Speed2, 'o')
ax.set_aspect('equal')
ax.set_xlim(-50,10)
ax.set_ylim(-50,10)
plt.plot(np.linspace(-50,10),np.linspace(-50,10),'r--')

#%%
