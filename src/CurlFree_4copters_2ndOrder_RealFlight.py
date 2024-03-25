#Flight of four copters
#by Tagir Muslimov, email: tagir.muslimov[at]gmail.com

from re import T
import time
import math
import logging

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

timestr = time.strftime("%Y%m%d-%H%M%S")
fp = open(timestr + '_NoAPF4Copt2ndOrder.csv', 'w')

URI1 = 'radio://0/80/2M/E7E7E7E711'
URI2 = 'radio://0/80/2M/E7E7E7E712'
URI3 = 'radio://0/80/2M/E7E7E7E713'
URI4 = 'radio://0/80/2M/E7E7E7E714'

# URI1 = 'radio://0/30/2M/E7E7E7E701'
# URI2 = 'radio://0/30/2M/E7E7E7E702'
# URI3 = 'radio://0/30/2M/E7E7E7E703'
# URI4 = 'radio://0/30/2M/E7E7E7E704'

uris = {
    # For the sim_cf2 simulation, the uris need to start at E7E7E7E701
    'radio://0/30/2M/E7E7E7E701',
    'radio://0/30/2M/E7E7E7E702',
    'radio://0/30/2M/E7E7E7E703',
    'radio://0/30/2M/E7E7E7E704'
    # Add more URIs if you want more copters in the swarm
}

MIN_BAT = 2.8
k_f = 15.5
k_f_v = 5.2
safety_radius = 0.4
T_Z = 0.3

a_axis_list = [[0,0],[0,0],[0,0],[0,0]]

logging.basicConfig(level=logging.ERROR)

position_estimate_cf1 = [0, 0, 0]
position_estimate_cf2 = [0, 0, 0]
position_estimate_cf3 = [0, 0, 0]
position_estimate_cf4 = [0, 0, 0]

velocity_estimate_cf1 = [0, 0, 0]
velocity_estimate_cf2 = [0, 0, 0]
velocity_estimate_cf3 = [0, 0, 0]
velocity_estimate_cf4 = [0, 0, 0]

# angle_estimate_cf1 = [0, 0, 0]
# angle_estimate_cf2 = [0, 0, 0]
# angle_estimate_cf3 = [0, 0, 0]
# angle_estimate_cf4 = [0, 0, 0]


def log_pos_callback_cf1(timestamp, data, logconf):
    global position_estimate_cf1
    global velocity_estimate_cf1
    global angle_estimate_cf1

    position_estimate_cf1[0] = data['kalman.stateX']
    position_estimate_cf1[1] = data['kalman.stateY']
    position_estimate_cf1[2] = data['kalman.stateZ']
    
    velocity_estimate_cf1[0] = data['stateEstimate.vx']
    velocity_estimate_cf1[1] = data['stateEstimate.vy']
    velocity_estimate_cf1[2] = data['stateEstimate.vz']

    # angle_estimate_cf1[0] = data['stateEstimate.roll']
    # angle_estimate_cf1[1] = data['stateEstimate.pitch']
    # angle_estimate_cf1[2] = data['stateEstimate.yaw']

    global lighthouse_status_cf1
    global bat_cf1

    # lighthouse_status_cf1 = data['lighthouse.status']
    # bat_cf1 = data['pm.vbat']


def log_pos_callback_cf2(timestamp, data, logconf):
    global position_estimate_cf2
    global velocity_estimate_cf2
    global angle_estimate_cf2

    position_estimate_cf2[0] = data['kalman.stateX']
    position_estimate_cf2[1] = data['kalman.stateY']
    position_estimate_cf2[2] = data['kalman.stateZ']

    velocity_estimate_cf2[0] = data['stateEstimate.vx']
    velocity_estimate_cf2[1] = data['stateEstimate.vy']
    velocity_estimate_cf2[2] = data['stateEstimate.vz']

    # angle_estimate_cf2[0] = data['stateEstimate.roll']
    # angle_estimate_cf2[1] = data['stateEstimate.pitch']
    # angle_estimate_cf2[2] = data['stateEstimate.yaw']

    global lighthouse_status_cf2
    global bat_cf2

    # lighthouse_status_cf2 = data['lighthouse.status']
    # bat_cf2 = data['pm.vbat']


def log_pos_callback_cf3(timestamp, data, logconf):
    global position_estimate_cf3
    global velocity_estimate_cf3
    global angle_estimate_cf3

    position_estimate_cf3[0] = data['kalman.stateX']
    position_estimate_cf3[1] = data['kalman.stateY']
    position_estimate_cf3[2] = data['kalman.stateZ']

    velocity_estimate_cf3[0] = data['stateEstimate.vx']
    velocity_estimate_cf3[1] = data['stateEstimate.vy']
    velocity_estimate_cf3[2] = data['stateEstimate.vz']

    # angle_estimate_cf3[0] = data['stateEstimate.roll']
    # angle_estimate_cf3[1] = data['stateEstimate.pitch']
    # angle_estimate_cf3[2] = data['stateEstimate.yaw']

    global lighthouse_status_cf3
    global bat_cf3

    # lighthouse_status_cf3 = data['lighthouse.status']
    # bat_cf3 = data['pm.vbat']

def log_pos_callback_cf4(timestamp, data, logconf):
    global position_estimate_cf4
    global velocity_estimate_cf4
    global angle_estimate_cf4

    position_estimate_cf4[0] = data['kalman.stateX']
    position_estimate_cf4[1] = data['kalman.stateY']
    position_estimate_cf4[2] = data['kalman.stateZ']

    velocity_estimate_cf4[0] = data['stateEstimate.vx']
    velocity_estimate_cf4[1] = data['stateEstimate.vy']
    velocity_estimate_cf4[2] = data['stateEstimate.vz']

    # angle_estimate_cf4[0] = data['stateEstimate.roll']
    # angle_estimate_cf4[1] = data['stateEstimate.pitch']
    # angle_estimate_cf4[2] = data['stateEstimate.yaw']

    global lighthouse_status_cf4
    global bat_cf4

    # lighthouse_status_cf4 = data['lighthouse.status']
    # bat_cf4 = data['pm.vbat']

def take_off(cf1, cf2, cf3, cf4, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position / take_off_time

    print(vz)

    for i in range(steps):
        print("take_off" + str(i))
        cf1.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf2.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf3.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf4.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def forward(cf, distance):
    sleep_time = 0.1
    vx = 0.1
    steps = int(distance / vx / sleep_time)

    for i in range(steps):
        print("forward" + str(i))
        cf.commander.send_velocity_world_setpoint(vx, 0, 0, 0)
        time.sleep(sleep_time)


def init_log(**log_vars):
    for k, v in log_vars.items():
        fp.write(str(k) + ';')
    fp.write('\n')


def write_log(**log_vars):
    for k, v in log_vars.items():
        fp.write(str(v) + ';')
    fp.write('\n')


def forward_CurlFree(cf1, cf2, cf3, cf4):
    steps = 700
    for i in range(steps):

        # print("forward_CurlFree" + str(i))
        # print(position_estimate_cf1)
        # print(position_estimate_cf2)
        # print(position_estimate_cf3)
        # print(position_estimate_cf4)

        px_1 = position_estimate_cf1[0]
        py_1 = position_estimate_cf1[1]
        pz_1 = position_estimate_cf1[2]
        print ("px_1:", px_1, "py_1:", py_1)

        px_2 = position_estimate_cf2[0]
        py_2 = position_estimate_cf2[1]
        pz_2 = position_estimate_cf2[2]
        print ("px_2:", px_2, "py_2:", py_2)

        px_3 = position_estimate_cf3[0]
        py_3 = position_estimate_cf3[1]
        pz_3 = position_estimate_cf3[2]
        print ("px_3:", px_3, "py_3:", py_3)

        px_4 = position_estimate_cf4[0]
        py_4 = position_estimate_cf4[1]
        pz_4 = position_estimate_cf4[2]
        print ("px_4:", px_4, "py_4:", py_4)

        vx_1 = velocity_estimate_cf1[0]
        vy_1 = velocity_estimate_cf1[1]
        vz_1 = velocity_estimate_cf1[2]

        vx_2 = velocity_estimate_cf2[0]
        vy_2 = velocity_estimate_cf2[1]
        vz_2 = velocity_estimate_cf2[2]

        vx_3 = velocity_estimate_cf3[0]
        vy_3 = velocity_estimate_cf3[1]
        vz_3 = velocity_estimate_cf3[2]

        vx_4 = velocity_estimate_cf4[0]
        vy_4 = velocity_estimate_cf4[1]
        vz_4 = velocity_estimate_cf4[2]
        
        # roll_1 = angle_estimate_cf1[0]
        # pitch_1 = angle_estimate_cf1[1]
        # yaw_1 = angle_estimate_cf1[2]

        # roll_2 = angle_estimate_cf2[0]
        # pitch_2 = angle_estimate_cf2[1]
        # yaw_2 = angle_estimate_cf2[2]

        # roll_3 = angle_estimate_cf3[0]
        # pitch_3 = angle_estimate_cf3[1]
        # yaw_3 = angle_estimate_cf3[2]

        # roll_4 = angle_estimate_cf4[0]
        # pitch_4 = angle_estimate_cf4[1]
        # yaw_4 = angle_estimate_cf4[2]

        distance_12 = distance_between_copters(px_1, py_1, px_2, py_2)
        distance_13 = distance_between_copters(px_1, py_1, px_3, py_3)
        distance_14 = distance_between_copters(px_1, py_1, px_4, py_4)
        distance_23 = distance_between_copters(px_2, py_2, px_3, py_3)
        distance_24 = distance_between_copters(px_2, py_2, px_4, py_4)
        distance_34 = distance_between_copters(px_3, py_3, px_4, py_4)

        copters_coordinates = [[px_1, py_1], [px_2, py_2], [px_3, py_3], [px_4, py_4]]
        copters_velocities  = [[vx_1, vy_1], [vx_2, vy_2], [vx_3, vy_3], [vx_4, vy_4]]

        break_out_flag = False
        for i in range(4):            
            for j in [x for x in range(4) if x != i]:
                # print ("i:", i, "j:", j)
                distance_ab = distance_between_copters(copters_coordinates[i][0], copters_coordinates[i][1], copters_coordinates[j][0], copters_coordinates[j][1])
                # print(copters[i][0], copters[i][1], copters[j][0], copters[j][1])
                # print(distance_ab)
                if distance_ab < safety_radius:
                    a_axis_list[i][0], a_axis_list[i][1] = axis_accel_APF(distance_ab, copters_coordinates[i][0], copters_coordinates[i][1], copters_coordinates[j][0], copters_coordinates[j][1],
                                                                             copters_velocities[i][0], copters_velocities[i][1], copters_velocities[j][0], copters_velocities[j][1]
                                                                             )
                    a_axis_list[j][0], a_axis_list[j][1] = axis_accel_APF(distance_ab, copters_coordinates[j][0], copters_coordinates[j][1], copters_coordinates[i][0], copters_coordinates[i][1],
                                                                             copters_velocities[j][0], copters_velocities[j][1], copters_velocities[i][0], copters_velocities[i][1]
                                                                             )
                    break_out_flag = True
                    break          
                   
            if break_out_flag:
                continue
            a_axis_list[i][0], a_axis_list[i][1] = axis_accel(i, px_1, py_1, px_2, py_2, px_3, py_3, px_4, py_4, vx_1, vy_1, vx_2, vy_2, vx_3, vy_3, vx_4, vy_4)

        ax1, ay1 = a_axis_list[0][0], a_axis_list[0][1]
        ax2, ay2 = a_axis_list[1][0], a_axis_list[1][1]
        ax3, ay3 = a_axis_list[2][0], a_axis_list[2][1]
        ax4, ay4 = a_axis_list[3][0], a_axis_list[3][1]
        print ("ax3, ay3:", ax3, ay3)

        setPx1 = px_1 + vx_1/10 + ax1/(2 * 100)
        setPx2 = px_2 + vx_2/10 + ax2/(2 * 100)
        setPx3 = px_3 + vx_3/10 + ax3/(2 * 100)
        setPx4 = px_4 + vx_4/10 + ax4/(2 * 100)

        setPy1 = py_1 + vy_1/10 + ay1/(2 * 100)
        setPy2 = py_2 + vy_2/10 + ay2/(2 * 100)
        setPy3 = py_3 + vy_3/10 + ay3/(2 * 100)
        setPy4 = py_4 + vy_4/10 + ay4/(2 * 100)
        print ("setPx3:", setPx3, "setPy3:", setPy3)

        if i == 0:
            # init_log(i=i, T_Z=T_Z, k_f=k_f, distance_12=distance_12, distance_13=distance_13, distance_14=distance_14, distance_23=distance_23, distance_24=distance_24, distance_34=distance_34,
            #          px_1=px_1, py_1=py_1, pz_1=pz_1, vx_1=vx_1, vy_1=vy_1,  vz_1=vz_1, roll_1=roll_1, pitch_1=pitch_1, yaw_1=yaw_1,
            #          px_2=px_2, py_2=py_2, pz_2=pz_2, vx_2=vx_2, vy_2=vy_2,  vz_2=vz_2, roll_2=roll_2, pitch_2=pitch_2, yaw_2=yaw_2,
            #          px_3=px_3, py_3=py_3, pz_3=pz_3, vx_3=vx_3, vy_3=vy_3,  vz_3=vz_3, roll_3=roll_3, pitch_3=pitch_3, yaw_3=yaw_3,
            #          px_4=px_4, py_4=py_4, pz_4=pz_4, vx_4=vx_4, vy_4=vy_4,  vz_4=vz_4, roll_4=roll_4, pitch_4=pitch_4, yaw_4=yaw_4,
            #          )

            init_log(i=i, T_Z=T_Z, k_f=k_f, k_f_v = k_f_v, safety_radius=safety_radius, distance_12=distance_12, distance_13=distance_13, distance_14=distance_14, distance_23=distance_23, distance_24=distance_24, distance_34=distance_34,
                     px_1=px_1, py_1=py_1, pz_1=pz_1, vx_1=vx_1, vy_1=vy_1,  vz_1=vz_1, 
                     px_2=px_2, py_2=py_2, pz_2=pz_2, vx_2=vx_2, vy_2=vy_2,  vz_2=vz_2, 
                     px_3=px_3, py_3=py_3, pz_3=pz_3, vx_3=vx_3, vy_3=vy_3,  vz_3=vz_3, 
                     px_4=px_4, py_4=py_4, pz_4=pz_4, vx_4=vx_4, vy_4=vy_4,  vz_4=vz_4, 
                     )

        # write_log(i=i, T_Z=T_Z, k_f=k_f, distance_12=distance_12, distance_13=distance_13, distance_14=distance_14, distance_23=distance_23, distance_24=distance_24, distance_34=distance_34,
        #              px_1=px_1, py_1=py_1, pz_1=pz_1, vx_1=vx_1, vy_1=vy_1, vz_1=vz_1, roll_1=roll_1, pitch_1=pitch_1, yaw_1=yaw_1,
        #              px_2=px_2, py_2=py_2, pz_2=pz_2, vx_2=vx_2, vy_2=vy_2, vz_2=vz_2, roll_2=roll_2, pitch_2=pitch_2, yaw_2=yaw_2,
        #              px_3=px_3, py_3=py_3, pz_3=pz_3, vx_3=vx_3, vy_3=vy_3, vz_3=vz_3, roll_3=roll_3, pitch_3=pitch_3, yaw_3=yaw_3,
        #              px_4=px_4, py_4=py_4, pz_4=pz_4, vx_4=vx_4, vy_4=vy_4, vz_4=vz_4, roll_4=roll_4, pitch_4=pitch_4, yaw_4=yaw_4,
        #              )
        
        write_log(i=i, T_Z=T_Z, k_f=k_f, k_f_v = k_f_v, safety_radius=safety_radius, distance_12=distance_12, distance_13=distance_13, distance_14=distance_14, distance_23=distance_23, distance_24=distance_24, distance_34=distance_34,
                     px_1=px_1, py_1=py_1, pz_1=pz_1, vx_1=vx_1, vy_1=vy_1, vz_1=vz_1, 
                     px_2=px_2, py_2=py_2, pz_2=pz_2, vx_2=vx_2, vy_2=vy_2, vz_2=vz_2, 
                     px_3=px_3, py_3=py_3, pz_3=pz_3, vx_3=vx_3, vy_3=vy_3, vz_3=vz_3, 
                     px_4=px_4, py_4=py_4, pz_4=pz_4, vx_4=vx_4, vy_4=vy_4, vz_4=vz_4, 
                     )        

        cf1.commander.send_position_setpoint(setPx1, setPy1, T_Z, 0)
        cf2.commander.send_position_setpoint(setPx2, setPy2, T_Z, 0)
        cf3.commander.send_position_setpoint(setPx3, setPy3, T_Z, 0)
        cf4.commander.send_position_setpoint(setPx4, setPy4, T_Z, 0)

        # if (bat_cf1 < MIN_BAT) or (bat_cf2 < MIN_BAT) or (bat_cf3 < MIN_BAT) or (bat_cf4 < MIN_BAT):
        #     print('BATTERY LOW: STOPPING')
        #     print('cf1:' +str(bat_cf1))
        #     print('cf2:' +str(bat_cf2))
        #     print('cf3:' +str(bat_cf3))
        #     print('cf4:' +str(bat_cf4))
        #     break

        time.sleep(0.1)


def land(cf1, cf2, cf3, cf4, position):
    landing_time = 1.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position / landing_time

    print(vz)

    for i in range(steps):
        print("land" + str(i))
        cf1.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf2.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf3.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        cf4.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf1.commander.send_stop_setpoint()
    cf2.commander.send_stop_setpoint()
    cf3.commander.send_stop_setpoint()
    cf4.commander.send_stop_setpoint()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)



def distance_between_copters(px_a, py_a, px_b, py_b):
    distance_ab=math.sqrt((px_a - px_b)**2 + (py_a - py_b)**2)
    return distance_ab

def axis_accel_APF(distance_ab, px_a, py_a, px_b, py_b, vx_a, vy_a, vx_b, vy_b):
        ax_APF_max = 0.1
        ay_APF_max = 0.1
        ############# APF with Curl Free Vector Field Modification:######################
        # steps = 250
        # for i in range(steps):
        ax_APF = 11.2 * (-1/safety_radius+1/distance_ab) * (px_a - px_b) / distance_ab**3 + 111*(vx_a - vx_b) - 110.1*(vy_a - vy_b)
        ay_APF = 11.2 * (-1/safety_radius+1/distance_ab) * (py_a - py_b) / distance_ab**3 + 111*(vy_a - vy_b) + 110.1*(vx_a - vx_b)
        # ax_APF = 12.2 * (-1/safety_radius+1/distance_ab) * (px_a - px_b) / distance_ab**3
        # ay_APF = 12.2 * (-1/safety_radius+1/distance_ab) * (py_a - py_b) / distance_ab**3
        ############# Standard APF:######################################################
        # ax_APF = 22 * (-1/safety_radius+1/distance_ab) * (px_a - px_b) / distance_ab**3
        # ay_APF = 22 * (-1/safety_radius+1/distance_ab) * (py_a - py_b) / distance_ab**3

        if ax_APF > ax_APF_max:
            ax_APF = ax_APF_max

        if ay_APF > ay_APF_max:
            ay_APF = ay_APF_max

        return (ax_APF, ay_APF)

def axis_accel(i, px1, py1, px2, py2, px3, py3, px4, py4, vx1_f, vy1_f, vx2_f, vy2_f, vx3_f, vy3_f, vx4_f, vy4_f):
        axa = 0 
        aya = 0
        axa_max = 0.1
        aya_max = 0.1
        if i==0:
            axa =  -k_f * (px1 - px2 - 0) - k_f_v * (vx1_f - vx2_f)
            aya =  -k_f * (py1 - py2 + 1) - k_f_v * (vy1_f - vy2_f) 
        elif i==1:
            axa =  -k_f * (-px1 + 2*px2 - px3 + 1) - k_f_v * (-vx1_f + 2*vx2_f - vx3_f) 
            aya =  -k_f * (-py1 + 2*py2 - py3 - 1) - k_f_v * (-vy1_f + 2*vy2_f - vy3_f) 
        elif i==2:
            axa =  -k_f * (-px2 + 2*px3 - px4 - 1) - k_f_v * (-vx2_f + 2*vx3_f - vx4_f) 
            aya =  -k_f * (-py2 + 2*py3 - py4 - 1) - k_f_v * (-vy2_f + 2*vy3_f - vy4_f)
        elif i==3:
            axa =  -k_f * (-px3 + px4 + 0) - k_f_v * (-vx3_f + vx4_f)
            aya =  -k_f * (-py3 + py4 + 1) - k_f_v * (-vy3_f + vy4_f) 
        
        if axa > axa_max:
            axa = axa_max

        if aya > aya_max:
            aya = aya_max

        return (axa, aya)    

def velocity(va, vb):
        v = math.sqrt((va)**2 + (vb)**2)
        return (v.real) 


if __name__ == '__main__':

    cflib.crtp.init_drivers(enable_sim_driver=True)

    with SyncCrazyflie(URI1, cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf1:
        with SyncCrazyflie(URI2, cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf2:
            with SyncCrazyflie(URI3, cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf3:
                with SyncCrazyflie(URI4, cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf4:
                    logconf1 = LogConfig(name='Position', period_in_ms=10)
                    logconf1.add_variable('kalman.stateX', 'float')
                    logconf1.add_variable('kalman.stateY', 'float')
                    logconf1.add_variable('kalman.stateZ', 'float')
                    logconf1.add_variable('stateEstimate.vx', 'float')
                    logconf1.add_variable('stateEstimate.vy', 'float')
                    logconf1.add_variable('stateEstimate.vz', 'float')
                    # logconf1.add_variable('stateEstimate.roll', 'float')
                    # logconf1.add_variable('stateEstimate.pitch', 'float')
                    # logconf1.add_variable('stateEstimate.yaw', 'float')
                    # logconf1.add_variable('pm.vbat', 'float')
                    # logconf1.add_variable('lighthouse.status', 'uint8_t')
                    scf1.cf.log.add_config(logconf1)
                    logconf1.data_received_cb.add_callback(log_pos_callback_cf1)

                    logconf2 = LogConfig(name='Position', period_in_ms=10)
                    logconf2.add_variable('kalman.stateX', 'float')
                    logconf2.add_variable('kalman.stateY', 'float')
                    logconf2.add_variable('kalman.stateZ', 'float')
                    logconf2.add_variable('stateEstimate.vx', 'float')
                    logconf2.add_variable('stateEstimate.vy', 'float')
                    logconf2.add_variable('stateEstimate.vz', 'float')
                    # logconf2.add_variable('stateEstimate.roll', 'float')
                    # logconf2.add_variable('stateEstimate.pitch', 'float')
                    # logconf2.add_variable('stateEstimate.yaw', 'float')
                    # logconf2.add_variable('pm.vbat', 'float')
                    # logconf2.add_variable('lighthouse.status', 'uint8_t')
                    scf2.cf.log.add_config(logconf2)
                    logconf2.data_received_cb.add_callback(log_pos_callback_cf2)

                    logconf3 = LogConfig(name='Position', period_in_ms=10)
                    logconf3.add_variable('kalman.stateX', 'float')
                    logconf3.add_variable('kalman.stateY', 'float')
                    logconf3.add_variable('kalman.stateZ', 'float')
                    logconf3.add_variable('stateEstimate.vx', 'float')
                    logconf3.add_variable('stateEstimate.vy', 'float')
                    logconf3.add_variable('stateEstimate.vz', 'float')
                    # logconf3.add_variable('stateEstimate.roll', 'float')
                    # logconf3.add_variable('stateEstimate.pitch', 'float')
                    # logconf3.add_variable('stateEstimate.yaw', 'float')
                    # logconf3.add_variable('pm.vbat', 'float')
                    # logconf3.add_variable('lighthouse.status', 'uint8_t')
                    scf3.cf.log.add_config(logconf3)
                    logconf3.data_received_cb.add_callback(log_pos_callback_cf3)

                    logconf4 = LogConfig(name='Position', period_in_ms=10)
                    logconf4.add_variable('kalman.stateX', 'float')
                    logconf4.add_variable('kalman.stateY', 'float')
                    logconf4.add_variable('kalman.stateZ', 'float')
                    logconf4.add_variable('stateEstimate.vx', 'float')
                    logconf4.add_variable('stateEstimate.vy', 'float')
                    logconf4.add_variable('stateEstimate.vz', 'float')
                    # logconf4.add_variable('stateEstimate.roll', 'float')
                    # logconf4.add_variable('stateEstimate.pitch', 'float')
                    # logconf4.add_variable('stateEstimate.yaw', 'float')
                    # logconf4.add_variable('pm.vbat', 'float')
                    # logconf4.add_variable('lighthouse.status', 'uint8_t')
                    scf4.cf.log.add_config(logconf4)
                    logconf4.data_received_cb.add_callback(log_pos_callback_cf4)

                    logconf1.start()
                    logconf2.start()
                    logconf3.start()
                    logconf4.start()

                    cf1 = scf1.cf
                    cf2 = scf2.cf
                    cf3 = scf3.cf
                    cf4 = scf4.cf

                    # take off
                    take_off(cf1, cf2, cf3, cf4, T_Z)

                    # flight
                    forward_CurlFree(cf1, cf2, cf3, cf4)

                    # landing
                    land(cf1, cf2, cf3, cf4, 0)

                    logconf1.stop()
                    logconf2.stop()
                    logconf3.stop()
                    logconf4.stop()

    fp.close()
