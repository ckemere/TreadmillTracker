#%%
import numpy as np
import seaborn as sns

#%%
#d = np.loadtxt('ClosedLoopData.txt')
d = np.loadtxt('IntereventDataLatency.txt')



#%%
sns.distplot(d[d>0.001],100)

#%%
sns.distplot(d[d<=0.001],100)

#%%
print(np.mean(d))
print(np.median(d))
print(np.std(d))

#%%
print(np.min(d))
print(np.max(d))
print(len(d))

#%%
