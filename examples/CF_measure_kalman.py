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



POSITION_X = []
POSITION_Y = []
POSITION_Z = []
TARGET_X = []
TARGET_Y = []
TARGET_Z = []
VX = []
VY = []
VZ = []
TARGET_VX = []
TARGET_VY = []
TARGET_VZ = []
ROLL = []
PITCH = []
YAW = []
CMD_ROLL = []
CMD_PITCH = []
CMD_YAW = []

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'

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
    (0, 0, h),
    (a, 0, h),
    (a, 0, h),
    (0, 0, h),
    #(0, -a, h),
    #(0, -a, h),
    #(0, 0, h, 0),
    (0, 0, 0.5*h),
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


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.0008 #0.001

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


def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf

    for position in sequence:
        print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for i in range(50):
            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

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


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_x = 0 #0
    initial_y = -0.1  #2.9
    initial_z =  0.1  #0
    initial_yaw = 0  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

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
        with open('/home/guillermoga/kalman_tests/crazyflie_kalman_' + datetime.now().strftime("%Y%m%d-%H%M%S") + '.txt', 'w') as f:
            print('Flight log' + datetime.now().strftime("%Y%m%d %H%M%S"), file=f)

            scf.cf.log.add_config(log_pos_estimate)
            log_pos_estimate.data_received_cb.add_callback(position_estimate_callback)
            scf.cf.log.add_config(log_vel_estimate)
            log_vel_estimate.data_received_cb.add_callback(velocity_estimate_callback)
            scf.cf.log.add_config(log_att_estimate)
            log_att_estimate.data_received_cb.add_callback(attitude_estimate_callback)

            set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
            reset_estimator(scf)

        
            log_pos_estimate.start()
            log_vel_estimate.start()
            log_att_estimate.start()

            run_sequence(scf, sequence, initial_x, initial_y, initial_z, initial_yaw)

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





   