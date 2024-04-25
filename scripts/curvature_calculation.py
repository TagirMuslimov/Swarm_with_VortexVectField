# Partly based on https://stackoverflow.com/questions/28269379/curve-curvature-in-numpy

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math

df = pd.read_csv('/home/tagir/Swarm_with_VortexVectField/data/Simulation/20240323-195835_Vortex4Copt2ndOrder.csv', header=0, delimiter=';')
dist_12=df['distance_12'].sum()
# print('dist_12:', dist_12)
err_12 = abs (dist_12 - 1000)

dist_23=df['distance_23'].sum()
# print('dist_23:', dist_23)
err_23 = abs (dist_23 - 1000)

dist_34=df['distance_34'].sum()
# print('dist_34:', dist_34)
err_34 = abs (dist_34 - 1000)

tot_sum = err_12 + err_23 + err_34 
# print('tot_sum:', tot_sum)

x_coordinate_1=df['px_1']
y_coordinate_1=df['py_1']
x_coordinate_2=df['px_2']
y_coordinate_2=df['py_2']
x_coordinate_3=df['px_3']
y_coordinate_3=df['py_3']
x_coordinate_4=df['px_4']
y_coordinate_4=df['py_4']


def curvature_calculation(x_coordinate_a, y_coordinate_a):
    dx_dt = np.gradient(x_coordinate_1)
    dy_dt = np.gradient(y_coordinate_1)
    velocity = np.array([ [dx_dt[i], dy_dt[i]] for i in range(dx_dt.size)])
    ds_dt = np.sqrt(dx_dt * dx_dt + dy_dt * dy_dt)
    # print(ds_dt)
    tangent = np.array([1/ds_dt]).transpose() * velocity
    # print(tangent)
    tttt = np.sqrt(tangent[:,0] * tangent[:,0] + tangent[:,1] * tangent[:,1])
    # print(tttt)
    tangent_x = tangent[:, 0]
    tangent_y = tangent[:, 1]

    deriv_tangent_x = np.gradient(tangent_x)
    deriv_tangent_y = np.gradient(tangent_y)

    dT_dt = np.array([ [deriv_tangent_x[i], deriv_tangent_y[i]] for i in range(deriv_tangent_x.size)])

    length_dT_dt = np.sqrt(deriv_tangent_x * deriv_tangent_x + deriv_tangent_y * deriv_tangent_y)

    normal = np.array([1/length_dT_dt]).transpose() * dT_dt
    # print (normal)

    d2s_dt2 = np.gradient(ds_dt)
    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)

    curvature_a = np.abs(d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / (dx_dt * dx_dt + dy_dt * dy_dt)**1.5
    # print (curvature_a)

    t_component = np.array([d2s_dt2] * 2).transpose()
    n_component = np.array([curvature_a * ds_dt * ds_dt] * 2).transpose()

    acceleration = t_component * tangent + n_component * normal
    
    return curvature_a

curvature_1 = curvature_calculation(x_coordinate_1, y_coordinate_1)
curvature_2 = curvature_calculation(x_coordinate_2, y_coordinate_2)
curvature_3 = curvature_calculation(x_coordinate_3, y_coordinate_3)
curvature_4 = curvature_calculation(x_coordinate_4, y_coordinate_4)

overall_curvature_sum = sum(curvature_1) + sum(curvature_2) + sum(curvature_3) + sum(curvature_4)
print(overall_curvature_sum)