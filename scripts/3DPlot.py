import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True
ax = plt.axes(projection='3d')

df = pd.read_csv('/home/tagir/Swarm_with_CurlFreeVectField/data/Simulation/20240323-195835_Curl4Copt2ndOrder.csv', header=0, delimiter=';')
cols = df.columns
for col in cols:
    df[col] = df[col].astype(float)

print(df.columns)

my_cmap = cm.plasma

x_1 = df['px_1']
y_1 = df['py_1']
z_1 = df['pz_1']
x_2 = df['px_2']
y_2 = df['py_2']
z_2 = df['pz_2']
x_3 = df['px_3']
y_3 = df['py_3']
z_3 = df['pz_3']
x_4 = df['px_4']
y_4 = df['py_4']
z_4 = df['pz_4']

plt.cla()

# fig, ax = plt.subplots()

plt_1 = ax.plot3D(x_1, y_1, z_1, label='Copter #1', c=my_cmap(0.09), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.09), markeredgecolor=my_cmap(1), markersize=18, linewidth=2)
plt_2 = ax.plot3D(x_2, y_2, z_2, label='Copter #2', c=my_cmap(0.38), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.38), markeredgecolor=my_cmap(1), markersize=18, linewidth=2)
plt_3 = ax.plot3D(x_3, y_3, z_3, label='Copter #3', c=my_cmap(0.62), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.62), markeredgecolor=my_cmap(1), markersize=18, linewidth=2)
plt_4 = ax.plot3D(x_4, y_4, z_4, label='Copter #4', c=my_cmap(0.8), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.8), markeredgecolor=my_cmap(1), markersize=18, linewidth=2)

ax.set_xlabel('East-West Position, [m]')
ax.set_ylabel('North-South Position, [m]')
ax.set_zlabel('Altitude, [m]')


plt.show()

# plt.savefig('position.eps')