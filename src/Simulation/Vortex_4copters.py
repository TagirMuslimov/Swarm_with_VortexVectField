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
fp = open(timestr + '_Vortex4Copt.csv', 'w')

# URI1 = 'radio://0/80/2M/E7E7E7E711'
# URI2 = 'radio://0/80/2M/E7E7E7E712'
# URI3 = 'radio://0/80/2M/E7E7E7E713'
# URI4 = 'radio://0/80/2M/E7E7E7E714'

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
k_f = 0.5
safety_radius = 0.4
T_Z = 0.3

v_axis_list = [[0,0],[0,0],[0,0],[0,0]]

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
    steps = 1100
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

        copters = [[px_1, py_1], [px_2, py_2], [px_3, py_3], [px_4, py_4]]

        break_out_flag = False
        for i in range(4):            
            for j in [x for x in range(4) if x != i]:
                # print ("i:", i, "j:", j)
                distance_ab = distance_between_copters(copters[i][0], copters[i][1], copters[j][0], copters[j][1])
                # print(copters[i][0], copters[i][1], copters[j][0], copters[j][1])
                # print(distance_ab)
                if distance_ab < safety_radius:
                    v_axis_list[i][0], v_axis_list[i][1] = axis_velocity_APF(distance_ab, copters[i][0], copters[i][1], copters[j][0], copters[j][1])
                    v_axis_list[j][0], v_axis_list[j][1] = axis_velocity_APF(distance_ab, copters[j][0], copters[j][1], copters[i][0], copters[i][1])
                    break_out_flag = True
                    break          
                   
            if break_out_flag:
                continue
            v_axis_list[i][0], v_axis_list[i][1] = axis_velocity(i, px_1, py_1, px_2, py_2, px_3, py_3, px_4, py_4)

        vx1, vy1 = v_axis_list[0][0], v_axis_list[0][1]
        vx2, vy2 = v_axis_list[1][0], v_axis_list[1][1]
        vx3, vy3 = v_axis_list[2][0], v_axis_list[2][1]
        vx4, vy4 = v_axis_list[3][0], v_axis_list[3][1]
        print ("vx3, vy3:", vx3, vy3)

        setPx1 = px_1 + vx1/10
        setPx2 = px_2 + vx2/10
        setPx3 = px_3 + vx3/10
        setPx4 = px_4 + vx4/10

        setPy1 = py_1 + vy1/10
        setPy2 = py_2 + vy2/10
        setPy3 = py_3 + vy3/10
        setPy4 = py_4 + vy4/10

        print ("setPx3:", setPx3, "setPy3:", setPy3)

        if i == 0:
            # init_log(i=i, T_Z=T_Z, k_f=k_f, distance_12=distance_12, distance_13=distance_13, distance_14=distance_14, distance_23=distance_23, distance_24=distance_24, distance_34=distance_34,
            #          px_1=px_1, py_1=py_1, pz_1=pz_1, vx_1=vx_1, vy_1=vy_1,  vz_1=vz_1, roll_1=roll_1, pitch_1=pitch_1, yaw_1=yaw_1,
            #          px_2=px_2, py_2=py_2, pz_2=pz_2, vx_2=vx_2, vy_2=vy_2,  vz_2=vz_2, roll_2=roll_2, pitch_2=pitch_2, yaw_2=yaw_2,
            #          px_3=px_3, py_3=py_3, pz_3=pz_3, vx_3=vx_3, vy_3=vy_3,  vz_3=vz_3, roll_3=roll_3, pitch_3=pitch_3, yaw_3=yaw_3,
            #          px_4=px_4, py_4=py_4, pz_4=pz_4, vx_4=vx_4, vy_4=vy_4,  vz_4=vz_4, roll_4=roll_4, pitch_4=pitch_4, yaw_4=yaw_4,
            #          )

            init_log(i=i, T_Z=T_Z, k_f=k_f, safety_radius=safety_radius, distance_12=distance_12, distance_13=distance_13, distance_14=distance_14, distance_23=distance_23, distance_24=distance_24, distance_34=distance_34,
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
        
        write_log(i=i, T_Z=T_Z, k_f=k_f, safety_radius=safety_radius, distance_12=distance_12, distance_13=distance_13, distance_14=distance_14, distance_23=distance_23, distance_24=distance_24, distance_34=distance_34,
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

def axis_velocity_APF(distance_ab, px_a, py_a, px_b, py_b):
        vx_APF_max = 0.2
        vy_APF_max = 0.2
        ############# APF with Curl Free Vector Field Modification:######################
        vx_APF = - 0.2 * (-1/safety_radius+1/distance_ab) * (py_a - py_b) / distance_ab**3 + 0.2*(px_a - px_b)
        vy_APF = 0.2 * (-1/safety_radius+1/distance_ab) * (px_a - px_b) / distance_ab**3 + 0.2*(py_a - py_b)
        ############# Standard APF:######################################################
        # vx_APF = 0.2 * (-1/safety_radius+1/distance_ab) * (px_a - px_b) / distance_ab**3
        # vy_APF = 0.2 * (-1/safety_radius+1/distance_ab) * (py_a - py_b) / distance_ab**3

        if vx_APF > vx_APF_max:
            vx_APF = vx_APF_max

        if vy_APF > vy_APF_max:
            vy_APF = vy_APF_max

        return (vx_APF, vy_APF)

def axis_velocity(i, px1, py1, px2, py2, px3, py3, px4, py4):
        vxa = 0 
        vya = 0
        vxa_max = 0.2
        vya_max = 0.2
        if i==0:
            vxa =  -k_f * (px1 - px2 - 0) 
            vya =  -k_f * (py1 - py2 + 1) 
        elif i==1:
            vxa =  -k_f * (-px1 + 2*px2 - px3 + 1) 
            vya =  -k_f * (-py1 + 2*py2 - py3 - 1) 
        elif i==2:
            vxa =  -k_f * (-px2 + 2*px3 - px4 - 1) 
            vya =  -k_f * (-py2 + 2*py3 - py4 - 1) 
        elif i==3:
            vxa =  -k_f * (-px3 + px4 + 0) 
            vya =  -k_f * (-py3 + py4 + 1) 
        
        if vxa > vxa_max:
            vxa = vxa_max

        if vya > vya_max:
            vya = vya_max

        return (vxa, vya)    

def velocity(va, vb):
        v = math.sqrt((va)**2 + (vb)**2)
        return (v.real) 


if __name__ == '__main__':

    cflib.crtp.init_drivers(enable_sim_driver=True)

    with SyncCrazyflie('radio://0/30/2M/E7E7E7E701', cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf1:
        with SyncCrazyflie('radio://0/30/2M/E7E7E7E702', cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf2:
            with SyncCrazyflie('radio://0/30/2M/E7E7E7E703', cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf3:
                with SyncCrazyflie('radio://0/30/2M/E7E7E7E704', cf=cflib.crazyflie.Crazyflie(rw_cache='./cache')) as scf4:
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
