import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib import cm


plt.rcParams["figure.autolayout"] = True

df = pd.read_csv('/home/tagir/Swarm_with_VortexVectField/data/Simulation/20240415-011823_Vortex4Copt2ndOrder.csv', header=0, delimiter=';')

cols = df.columns
for col in cols:
    df[col] = df[col].astype(float)

print(df.columns)

my_cmap = cm.plasma

# df.plot(x='i', y=['setPx1','setPx2','setPx3'])
# df.plot(x='i', y=['setPy1','setPy2','setPy3'])
# df.plot(x='i', y=['p_12', 'p_23'])
# ax = df.plot(x='i', y=['distance_12', 'distance_13','distance_14','distance_23','distance_24', 'distance_34'], markevery=[-1], marker='o', markerfacecolor='r', markeredgecolor='r', title = 'Distances')
ax = df.plot(x='i', y=['distance_12', 'distance_13','distance_14','distance_23','distance_24', 'distance_34'])
ax.set(xlabel='Time, [ds]', ylabel='Distance between UAVs, [m]')

ax = df.plot(x='i', y=['vx_1', 'vy_1', 'vx_2', 'vy_2', 'vx_3', 'vy_3', 'vx_4', 'vy_4'])
ax.set(xlabel='Time, [ds]', ylabel='Velocities of UAVs, [m]')

# ax = df.plot(x='i', y=['distance_12'])
# ax.set(xlabel='Time, [ds]', ylabel='Distance between UAVs, [rad]')

# df.plot(x='i', y=['d_1','d_2', 'd_3'])
fig, ax = plt.subplots()
plt_1, = ax.plot(df['px_1'], df['py_1'], label='Copter #1', c=my_cmap(0.09), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.09), markeredgecolor=my_cmap(1), markersize=18)
plt_2, = ax.plot(df['px_2'], df['py_2'], label='Copter #2', c=my_cmap(0.38), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.38), markeredgecolor=my_cmap(1), markersize=18)
plt_3, = ax.plot(df['px_3'], df['py_3'], label='Copter #3', c=my_cmap(0.62), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.62), markeredgecolor=my_cmap(1), markersize=18)
plt_4, = ax.plot(df['px_4'], df['py_4'], label='Copter #4', c=my_cmap(0.8), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.8), markeredgecolor=my_cmap(1), markersize=18)
ax.legend()
ax.set(xlabel='East-West Position, [m]', ylabel='North-South Position, [m]', title = 'Trajectories')
# ax.legend([plt_1, plt_2], ['label1', 'label2'])



# df.plot(x='px_1', y=['py_1'])
# df.plot(x='px_2', y=['py_2'])
# ax = df.plot(x='px_1', y=['py_1'])
# df.plot(x='px_2', y=['py_2'], ax=ax)

###############################################
# fig, axes = plt.subplots(nrows=3, ncols=1)
# ax=df.plot(x='px_1', y=['py_1'], ax=axes[0])
# ax.set(xlabel='px_1, [m]', ylabel='py_1, [m]')

# ax=df.plot(x='px_2', y=['py_2'], ax=axes[1])
# ax.set(xlabel='px_2, [m]', ylabel='py_2, [m]')

# ax=df.plot(x='px_3', y=['py_3'], ax=axes[2])
# ax.set(xlabel='px_3, [m]', ylabel='py_3, [m]')

# df.plot(x='setPx3', y=['setPy3'])
plt.show()