#%%
import numpy as np
import seaborn as sns

#%%
d = np.loadtxt('ClosedLoopData.txt')

#%%
sns.distplot(d)

#%%
print(np.mean(d))
print(np.median(d))
print(np.std(d))

#%%
print(np.min(d))
print(np.max(d))
print(len(d))

#%%
