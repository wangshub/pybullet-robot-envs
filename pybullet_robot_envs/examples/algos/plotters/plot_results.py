import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print(currentdir)
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import baselines
from baselines.common import plot_util as pu
import matplotlib.pyplot as plt
import numpy as np

results = pu.load_results('')
r = results[0]
print(r.monitor.r)

plt.plot(np.cumsum(r.monitor.l), pu.smooth(r.monitor.r, radius=10))
plt.show()
