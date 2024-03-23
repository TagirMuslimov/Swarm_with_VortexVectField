import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True
ax = plt.axes(projection='3d')

# headers = ['i', 'T_Z', 'v_z', 'CX', 'CY', 'k', 'R', 'D_12', 'v_f', 'v_cruis', 'k_f', 'p12', 'px_1', 'py_1', 'pz_1', 'd_1', 'phi_1', 'angle_1', 'v1', 'vx1', 'vy1', 'setPx1', 'setPy1', 'px_2', 'py_2', 'pz_2', 'd_2', 'phi_2', 'angle_2', 'v2', 'vx2', 'vy2', 'setPx2', 'setPy2']

# df = pd.read_csv('/home/tagir/CircularMotion/20221014-140915_setpos2.csv', 
#             names=headers, header=0, delimiter=';',
#             skip_blank_lines=True,  engine='python'         
#             )

# cols = df.columns
# for col in cols:
#     df[col] = df[col].astype(float)
# df.set_index('i').plot()


# df = pd.read_csv(r'C:\Users\tagir\Documents\![Crazyflie Flight Tests]\![From Github]\CircularMotion\data\most valuable\3 copters with a moving center\sim3_20240302-203117_NoCurl.csv', header=0, delimiter=';')
df = pd.read_csv('/home/tagir/Simulation/20240323-190753_CurlFree4Copt.csv', header=0, delimiter=';')
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

plt_1 = ax.plot3D(x_1, y_1, z_1, label='Copter #1', c=my_cmap(0.09), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.09), markeredgecolor=my_cmap(0.09), markersize=9, linewidth=2)
plt_2 = ax.plot3D(x_2, y_2, z_2, label='Copter #2', c=my_cmap(0.38), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.38), markeredgecolor=my_cmap(0.38), markersize=9, linewidth=2)
plt_3 = ax.plot3D(x_3, y_3, z_3, label='Copter #3', c=my_cmap(0.62), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.62), markeredgecolor=my_cmap(0.62), markersize=9, linewidth=2)
plt_4 = ax.plot3D(x_4, y_4, z_4, label='Copter #4', c=my_cmap(0.8), markevery=[-1], marker='o', markerfacecolor=my_cmap(0.8), markeredgecolor=my_cmap(0.8), markersize=9, linewidth=2)

ax.set_xlabel('East-West Position, [m]')
ax.set_ylabel('North-South Position, [m]')
ax.set_zlabel('Altitude, [m]')


plt.show()

# plt.savefig('position.eps')