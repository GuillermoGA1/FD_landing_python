# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to one crazyflie, sets the initial position/yaw
and flies a trajectory.

The initial pose (x, y, z, yaw) is configured in a number of variables and
the trajectory is flown relative to this position, using the initial yaw.

This example is intended to work with any absolute positioning system.
It aims at documenting how to take off with the Crazyflie in an orientation
that is different from the standard positive X orientation and how to set the
initial position of the kalman estimator.
"""
import math
import sys
import time
import csv
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from datetime import datetime

import threading as th

kill_flight = False
def key_capture_thread():
    global kill_flight
    input()
    kill_flight = True

drone = 'CF'
# drone = 'FlapperRoadrunner'
drone = 'Flapper'

# Adress of the drone
if (drone == 'Flapper'):
    address = 'E7E7E7E7E3' # NimbleFlapper 2.0 Bolt
elif (drone == 'FlapperRoadrunner'):
    address = 'E7E7E7E7E0' # NimbleFlapper 2.0 Roadrunner
elif (drone == 'CF'):
    address = 'E7E7E7E7E3' # CF2.1
else:
    sys.exit()

# URI to the drone to connect to
uri = 'radio://0/80/2M/' + address

POSITION_X = []
POSITION_Y = []
POSITION_Z = []
X_VAR = []
Y_VAR = []
Z_VAR = []

VX = []
VY = []
VZ = []
VX_VAR = []
VY_VAR = []
VZ_VAR = []

D0 = []
D1 = []
D2 = []
D0_VAR = []
D1_VAR = []
D2_VAR = []

# Change the sequence according to your setup
#             x    y    z

a = 1.5 #length of side
h = 1 #height
sequence = [
    #square
    #(0, 0, h),
    #(a, 0, h),
    #(a, a, h),
    #(0, a, h),
    #(0, 0, h),
    #(0, 0, 0.3*h),
    #(0 ,0 ,0),
    #I-shape
    #    (0, 0, h),
    #    (0, a, h),
    #    (a, a, h),
    #    (a, -2*a, h),
    #    (0, -2*a, h),
    #    (0, -a, h),
    #    (-a, -a, h),
    #    (-a, -2*a, h),
    #   (-2*a, -2*a, h),
    #    (-2*a, a, h),
    #    (-a, a, h),
    #    (-a, 0, h),
    #    (0, 0, h),
    #    (0, 0, 0.5),
    #    (0, 0, 0),
    #Spiral
    #    (0, 0, h),
    #    (0, a, h),
    #    (1.5*a, a, h),
    #    (1.5*a, -2*a, h),
    #    (-1.5*a, -2*a, h),
    #    (-1.5*a, a, h),
    #    (-a, a, h),
    #    (-a, 0, h),
    #    (0, 0, h),
    #    (0, 0, 0.5),
    #    (0, 0 ,0),
    #hexagon
    #    (0, 0, h),
    #    (0.866*a, -0.5*a, h),
    #    (0.866*a, -1.5*a, h),
    #    (0, -2*a, h),
    #    (-0.866*a, -1.5*a, h),
    #    (-0.866*a, -0.5*a, h),
    #    (0, 0, h),
    #    (0, 0, 0.5),
    #    (0, 0, 0),
    #rhombus
    #    (0, 0, h),
    #    (-1.5*a, -1.5*a, h),
    #    (0, -3*a, h),
    #    (1.5*a, -1.5*a, h),
    #    (0, 0, h),
    #    (0, 0, 0.5),
    #    (0, 0, 0),
    #hover
    #(0, 0, 0.5*h, 0),
    (0, 0, h, 0),
    #(a, 0, h, 0),
    #(a, 0, h, 0),
    (0, 0, h, 0),
    #(0, -a, h),
    #(0, -a, h),
    #(0, 0, h, 0),
    (0, 0, 0.5*h, 0),
    #(0, 0, h),
    #(0, 0, h),
    #(0, 0, 0),
    #asterisk
    #    (0, 0, h),'
    #    (0, -a ,h),
    #    (-a, a, h),
    #    (a, 0, h),
    #    (-a, -a, h),
    #    (0, a, h),
    #    (a, -a, h),
    #    (-a, 0, h),
    #    (a, a, h),
    #    (0, 0, h),
    #    (0, 0, 0.5),
    #    (0, 0, 0),
]

origin = [0, -0.1, 0.1]

if (drone == 'Flapper') or (drone == 'FlapperRoadrunner'):
    # Setting for Nimble Flapper
    hover_thrust = 40000

elif (drone == 'CF'):
    # Setting for CF2.1
    hover_thrust = 35000



def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001  #0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break



def set_control_parameters_Bolt(scf):
    print('Setting control parameters for Nimble Flapper Bolt')


    #Low pass filters
    scf.cf.param.set_value('attFilt.rateFiltEn', '1') #1
    scf.cf.param.set_value('attFilt.omxFiltCut', '12.5') #12.5
    scf.cf.param.set_value('attFilt.omyFiltCut', '12.5') #12.5
    scf.cf.param.set_value('attFilt.omzFiltCut', '5.0') #5.0
    scf.cf.param.set_value('attFilt.attFiltEn', '0') #0
    scf.cf.param.set_value('attFilt.attFiltCut', '15.0') #15.0

    # # Double loop in-body control
    scf.cf.param.set_value('posCtlPid.singleLoop', '0')
    scf.cf.param.set_value('posVelFilt.posFiltEn', '0')
    scf.cf.param.set_value('posVelFilt.posFiltCut', '5.0')
    scf.cf.param.set_value('posVelFilt.velFiltEn', '0')
    scf.cf.param.set_value('posVelFilt.velFiltCut', '10.0')
    scf.cf.param.set_value('posVelFilt.posZFiltEn', '0')
    scf.cf.param.set_value('posVelFilt.posZFiltCut', '5.0')
    scf.cf.param.set_value('posVelFilt.velZFiltEn', '1')
    scf.cf.param.set_value('posVelFilt.velZFiltCut', '10.0')
    scf.cf.param.set_value('posCtlPid.thrustBase', hover_thrust)
    scf.cf.param.set_value('posCtlPid.thrustMin', '15000')

    #POSITION
    scf.cf.param.set_value('posCtlPid.xKp', '9.5') #4 6 test_good: 9.5
    scf.cf.param.set_value('posCtlPid.xKi', '0') #0 test_good: 0
    scf.cf.param.set_value('posCtlPid.xKd', '1.0') #0   test_good 1.0
    scf.cf.param.set_value('posCtlPid.yKp', '5.5') #2.5
    scf.cf.param.set_value('posCtlPid.yKi', '0') #0
    scf.cf.param.set_value('posCtlPid.yKd', '0') #0 
    scf.cf.param.set_value('posCtlPid.zKp', '5.0') #5 
    scf.cf.param.set_value('posCtlPid.zKi', '0.5') #0.5
    scf.cf.param.set_value('posCtlPid.zKd', '0.0') #0
    scf.cf.param.set_value('posCtlPid.rLimit', '45.0') #25
    scf.cf.param.set_value('posCtlPid.pLimit', '40.0') #25

    #VELOCITY
    scf.cf.param.set_value('velCtlPid.vxKFF', '2.0') #0 tested: 0.7
    scf.cf.param.set_value('velCtlPid.vxKp', '6.5') #4   tested: 6
    scf.cf.param.set_value('velCtlPid.vxKi', '1.0') #0.5 tested:0.8
    scf.cf.param.set_value('velCtlPid.vxKd', '0') #0 tested: 0.2
    scf.cf.param.set_value('velCtlPid.vyKFF', '1.0') #0
    scf.cf.param.set_value('velCtlPid.vyKp', '5.0') #4 tested:5
    scf.cf.param.set_value('velCtlPid.vyKi', '0.5') #0.5 tested: 0.9
    scf.cf.param.set_value('velCtlPid.vyKd', '0') #0  tested: 0.2
    scf.cf.param.set_value('velCtlPid.vzKp', '12.5') #12.5
    scf.cf.param.set_value('velCtlPid.vzKi', '5.0') #5
    scf.cf.param.set_value('velCtlPid.vzKd', '0.0') #0

    #MAX VELOCITIES
    scf.cf.param.set_value('posCtlPid.xBodyVelMax', '1.0') #3
    scf.cf.param.set_value('posCtlPid.yBodyVelMax', '1.0') #3
    scf.cf.param.set_value('posCtlPid.zVelMax', '1.0') #3

    #PITCH, ROLL & YAW 
    scf.cf.param.set_value('pid_attitude.yawFeedForw', '0') #220
    scf.cf.param.set_value('pid_attitude.pitch_kp', '15.0') #13  test_good:15
    scf.cf.param.set_value('pid_attitude.pitch_ki', '12.5')  #0  test:12.5
    scf.cf.param.set_value('pid_attitude.pitch_kd', '1')  #1    test:1 
    scf.cf.param.set_value('pid_attitude.roll_kp', '10.0')  #15   test:10
    scf.cf.param.set_value('pid_attitude.roll_ki', '1')   #0    test:1
    scf.cf.param.set_value('pid_attitude.roll_kd', '0.2')   #1    test:0.2
    scf.cf.param.set_value('pid_attitude.yaw_kp', '15.0')   #30   test:45
    scf.cf.param.set_value('pid_attitude.yaw_ki', '10.0')   #0    test:20
    scf.cf.param.set_value('pid_attitude.yaw_kd', '1.0')    #1    test:1

    #ATTITUDE RATES
    scf.cf.param.set_value('pid_rate.pitch_kp', '70.0')  #70
    scf.cf.param.set_value('pid_rate.pitch_ki', '0')  
    scf.cf.param.set_value('pid_rate.pitch_kd', '0') 
    scf.cf.param.set_value('pid_rate.roll_kp', '50.0')  #50
    scf.cf.param.set_value('pid_rate.roll_ki', '0')   
    scf.cf.param.set_value('pid_rate.roll_kd', '0')   
    scf.cf.param.set_value('pid_rate.yaw_kp', '80.0')   #80 
    scf.cf.param.set_value('pid_rate.yaw_ki', '0')   
    scf.cf.param.set_value('pid_rate.yaw_kd', '0')   


def set_control_parameters_CF(scf):
    print('Setting control parameters for Crazyflie')
    scf.cf.param.set_value('posCtlPid.rpLimit', '30.0')
    scf.cf.param.set_value('posCtlPid.xyVelMax', '3.0')
    scf.cf.param.set_value('posCtlPid.zVelMax', '3.0')

def set_initial_position(scf, x, y, z, yaw):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = yaw
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def reset_tumble_detector(scf):
    cf = scf.cf
    cf.param.set_value('stabilizer.stop', '0')
    time.sleep(0.1)

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def reinitialize_controller(scf):
    cf = scf.cf
    cf.param.set_value('stabilizer.controller', '0') # switch to ANY
    time.sleep(0.1)
    cf.param.set_value('stabilizer.controller', '1') # switch to PID
    time.sleep(0.1)

def log_async_setup(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_async_callback)

def log_and_print_async_setup(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_and_print_async_callback)

def log_async_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    if 'pm.vbat' in data:
        global vbat
        vbat = data['pm.vbat']
    #if 'stateEstimate.x' in data:
    #    data["timestamp"] = timestamp
    #    writer = csv.DictWriter(open('/home/guillermoga/flight_log.csv', 'a+', newline=''), fieldnames=["timestamp","stateEstimate.x","stateEstimate.y","stateEstimate.z"])
    #    writer.writerow(data)

def log_and_print_async_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    print('[%d]: %s' % (timestamp, data))


def position_estimate_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    position_x = data["kalman.stateX"]
    position_y = data["kalman.stateY"]
    position_z = data["kalman.stateZ"]
    var_x = data["kalman.varX"]
    var_y = data["kalman.varY"]
    var_z = data["kalman.varZ"]
    POSITION_X.append(position_x)
    POSITION_Y.append(position_y)
    POSITION_Z.append(position_z)
    X_VAR.append(var_x)
    Y_VAR.append(var_y)
    Z_VAR.append(var_z)

def velocity_estimate_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    velocity_x = data["kalman.statePX"]
    velocity_y = data["kalman.statePY"]
    velocity_z = data["kalman.statePZ"]
    var_velocity_x = data["kalman.varPX"]
    var_velocity_y = data["kalman.varPY"]
    var_velocity_z = data["kalman.varPZ"]
    VX.append(velocity_x)
    VY.append(velocity_y)
    VZ.append(velocity_z)
    VX_VAR.append(var_velocity_x)
    VY_VAR.append(var_velocity_y)
    VZ_VAR.append(var_velocity_z)

def attitude_estimate_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    roll = data["kalman.stateD0"]
    pitch = data["kalman.stateD1"]
    yaw = data["kalman.stateD2"]
    var_roll = data["kalman.varD0"]
    var_pitch = data["kalman.varD1"]
    var_yaw = data["kalman.varD2"]
    D0.append(roll)
    D1.append(pitch)
    D2.append(yaw)
    D0_VAR.append(var_roll)
    D1_VAR.append(var_pitch)
    D2_VAR.append(var_yaw)


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_x = 0
    initial_y = -0.1
    initial_z = 0.1
    initial_yaw = 0  # In degrees
    landing_yaw = 0
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    # define logging parameters

    log_vbat = LogConfig(name='pm', period_in_ms=500)
    log_vbat.add_variable('pm.vbat', 'float')

    log_pos_estimate = LogConfig(name='kalman', period_in_ms=50)
    log_pos_estimate.add_variable('kalman.stateX', 'float')
    log_pos_estimate.add_variable('kalman.stateY', 'float')
    log_pos_estimate.add_variable('kalman.stateZ', 'float')
    log_pos_estimate.add_variable('kalman.varX', 'float')
    log_pos_estimate.add_variable('kalman.varY', 'float')
    log_pos_estimate.add_variable('kalman.varZ', 'float')

    log_vel_estimate = LogConfig(name='kalman', period_in_ms=50)
    log_vel_estimate.add_variable('kalman.statePX', 'float')
    log_vel_estimate.add_variable('kalman.statePY', 'float')
    log_vel_estimate.add_variable('kalman.statePZ', 'float')
    log_vel_estimate.add_variable('kalman.varPX', 'float')
    log_vel_estimate.add_variable('kalman.varPY', 'float')
    log_vel_estimate.add_variable('kalman.varPZ', 'float')

    log_att_estimate = LogConfig(name='kalman', period_in_ms=50)
    log_att_estimate.add_variable('kalman.stateD0', 'float')
    log_att_estimate.add_variable('kalman.stateD1', 'float')
    log_att_estimate.add_variable('kalman.stateD2', 'float')
    log_att_estimate.add_variable('kalman.varD0', 'float')
    log_att_estimate.add_variable('kalman.varD1', 'float')
    log_att_estimate.add_variable('kalman.varD2', 'float')


    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with open('/home/guillermoga/kalman_tests/kalman_measures_' + datetime.now().strftime("%Y%m%d-%H%M%S") + '.txt', 'w') as f:
            print('Flight log' + datetime.now().strftime("%Y%m%d %H%M%S"), file=f)

            scf.cf.log.add_config(log_pos_estimate)
            log_pos_estimate.data_received_cb.add_callback(position_estimate_callback)
            scf.cf.log.add_config(log_vel_estimate)
            log_vel_estimate.data_received_cb.add_callback(velocity_estimate_callback)
            scf.cf.log.add_config(log_att_estimate)
            log_att_estimate.data_received_cb.add_callback(attitude_estimate_callback)
           
            log_async_setup(scf, log_vbat)
            log_vbat.start()

            cf = scf.cf

            time.sleep(1)
            print('Battery voltage is {}' .format(vbat))

            reset_tumble_detector(scf)

            if (drone == 'Flapper'):
                set_control_parameters_Bolt(scf)
            elif (drone == 'CF'):
                set_control_parameters_CF(scf)

            # reinitialize_controller(scf) # to load all new control parameters

            set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
            reset_estimator(scf)

            log_pos_estimate.start()
            log_vel_estimate.start()
            log_att_estimate.start()
     
            # unlock the engines
            # cf.commander.send_setpoint(0, 0, 0, 0)

            activate_high_level_commander(cf)
            time.sleep(0.2)
            commander = cf.high_level_commander

            commander.takeoff(-0.5, 0.001) # setting the setpoint underground to spinup the motors before taking off
            time.sleep(0.01)
            commander.go_to(origin[0], origin[1], origin[2]-0.5, initial_yaw, 0.01) # (re)set the setpoint, sometimes xy stays at [0,0]
            time.sleep(0.2)
            commander.go_to(origin[0], origin[1], origin[2], initial_yaw, 1.5)
            time.sleep(0.2)

            for position in sequence:
                print('Setting position {}'.format(position))

                x = position[0] + initial_x
                y = position[1] + initial_y
                z = position[2] + initial_z
                yaw = position[3]

                commander.go_to(x, y, z, yaw, 4)
                time.sleep(4)



            #commander.go_to(initial_x, initial_y, initial_z, 0.0, 2)
            #time.sleep(2.0)
            commander.land(initial_z - 0.5, 3.0, yaw= landing_yaw)
            time.sleep(3.0)
            commander.stop()

            log_vbat.stop()
            log_pos_estimate.stop()
            log_vel_estimate.stop()
            log_att_estimate.stop()
    
            fig, axs = plt.subplots(3, 3)
            axs[0, 0].plot(POSITION_X)
            axs[0, 0].plot(X_VAR)
            axs[0, 0].grid()
            axs[0, 0].legend(["est", "var"])
            axs[0, 0].set_title("Position X")
            axs[1, 0].plot(POSITION_Y )
            axs[1, 0].plot(Y_VAR)
            axs[1, 0].grid()
            axs[1, 0].legend(["est", "var"])
            axs[1, 0].set_title("Position Y")
            axs[2, 0].plot(POSITION_Z)
            axs[2, 0].plot(Z_VAR)
            axs[2, 0].grid()
            axs[2, 0].legend(["est", "var"])
            axs[2, 0].set_title("Position Z")
            axs[0, 1].plot(VX)
            axs[0, 1].plot(VX_VAR)
            axs[0, 1].grid()
            axs[0, 1].legend(["est", "var"])
            axs[0, 1].set_title("Velocity X")
            axs[1, 1].plot(VY)
            axs[1, 1].plot(VY_VAR)
            axs[1, 1].grid()
            axs[1, 1].legend(["est", "var"])
            axs[1, 1].set_title("Velocity Y")
            axs[2, 1].plot(VZ)
            axs[2, 1].plot(VZ_VAR)
            axs[2, 1].grid()
            axs[2, 1].legend(["est", "var"])
            axs[2, 1].set_title("Velocity Z")
            axs[0, 2].plot(D0)
            axs[0, 2].plot(D0_VAR)
            axs[0, 2].grid()
            axs[0, 2].legend(["est", "var"])
            axs[0, 2].set_title("Pitch error")
            axs[1, 2].plot(D1)
            axs[1, 2].plot(D1_VAR)
            axs[1, 2].grid()
            axs[1, 2].legend(["est", "var"])
            axs[1, 2].set_title("Roll_error")
            axs[2, 2].plot(D2)
            axs[2, 2].plot(D2_VAR)
            axs[2, 2].grid()
            axs[2, 2].legend(["est", "var"])
            axs[2, 2].set_title("Yaw_error")
            plt.show()

