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
    address = 'E7E7E7E7E7' # NimbleFlapper 2.0 Bolt
elif (drone == 'FlapperRoadrunner'):
    address = 'E7E7E7E7E0' # NimbleFlapper 2.0 Roadrunner
elif (drone == 'CF'):
    address = 'E7E7E7E7E7' # CF2.1
else:
    sys.exit()

# URI to the drone to connect to
uri = 'radio://0/80/2M/' + address

POSITION_X = []
POSITION_Y = []
POSITION_Z = []
TARGET_X = []
TARGET_Y = []
TARGET_Z = []
ROLL = []
PITCH = []
YAW = []
CMD_ROLL = []
CMD_PITCH = []
CMD_YAW = []
# Change the sequence according to your setup
#             x    y    z

a = 1.5 #length of side
h = 1.2 #height
sequence = [
    #square
    #    (0, 0, h),
    #    (0, a, h),
    #    (a, a, h),
    #    (a,-2*a, h),
    #    (-2*a,-2*a, h),
    #    (-a, -a, h),
    #    (-a, a, h),
    #    (0, a, h),
    #    (0, 0, h),
    #    (0, 0, 0.5),
    #    (0, 0, 0),
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
    (0, 0, 0.5*h),
    (0, 0, h),
    (a, 0, h),
    (a, 0, 0.5*h),
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

origin = [0, 0, -0.3]

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

    threshold = 0.0008

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


def set_control_parameters_Roadrunner(scf):
    print('Setting control parameters for Nimble Flapper Roadrunner')

    scf.cf.param.set_value('attFilt.rateFiltEn', '1') #1
    scf.cf.param.set_value('attFilt.omxFiltCut', '12.5') #12.5
    scf.cf.param.set_value('attFilt.omyFiltCut', '12.5') #12.5
    scf.cf.param.set_value('attFilt.omzFiltCut', '5.0') #5.0

    scf.cf.param.set_value('attFilt.attFiltEn', '0') #0
    scf.cf.param.set_value('attFilt.attFiltCut', '15.0') #15.0

    # scf.cf.param.set_value('pid_attitude.roll_kp', '15.0') #15
    # scf.cf.param.set_value('pid_attitude.roll_kd', '1.0') #1
    # scf.cf.param.set_value('pid_rate.roll_kp', '50.0') #50
    # scf.cf.param.set_value('pid_attitude.pitch_kp', '13.0') #13
    # scf.cf.param.set_value('pid_attitude.pitch_kd', '1.0') #1
    # scf.cf.param.set_value('pid_rate.pitch_kp', '70.0') #70
    # scf.cf.param.set_value('pid_attitude.yaw_kp', '30.0') #30
    # scf.cf.param.set_value('pid_attitude.yaw_kd', '1.0') #1
    # scf.cf.param.set_value('pid_rate.yaw_kp', '100.0') #100
    # scf.cf.param.set_value('pid_attitude.yawFeedForw', '220.0') #220.0

    # # Single loop in-body control
    # scf.cf.param.set_value('posCtlPid.singleLoop', '1')
    # scf.cf.param.set_value('posVelFilt.posFiltEn', '1')
    # scf.cf.param.set_value('posVelFilt.posFiltCut', '7.0')
    # scf.cf.param.set_value('posCtlPid.thrustBase', hover_thrust)
    # scf.cf.param.set_value('posCtlPid.thrustMin', '20000')
    # scf.cf.param.set_value('posCtlPid.xKp', '32.0') #32
    # scf.cf.param.set_value('posCtlPid.xKi', '2.0')
    # scf.cf.param.set_value('posCtlPid.xKd', '8.0') #8
    # scf.cf.param.set_value('posCtlPid.yKp', '25.0') #25
    # scf.cf.param.set_value('posCtlPid.yKi', '2.0')
    # scf.cf.param.set_value('posCtlPid.yKd', '10.0') #10
    # scf.cf.param.set_value('posCtlPid.zKp', '62.5')
    # scf.cf.param.set_value('posCtlPid.zKi', '6.0')
    # scf.cf.param.set_value('posCtlPid.zKd', '12.6')
    # scf.cf.param.set_value('posCtlPid.rpLimit', '20.0')

    # Double loop in-body control - initial parameters
    # scf.cf.param.set_value('posCtlPid.thrustBase', hover_thrust)
    # scf.cf.param.set_value('posCtlPid.thrustMin', '20000')
    # scf.cf.param.set_value('posCtlPid.xKp', '4.0')
    # scf.cf.param.set_value('posCtlPid.xKi', '0.25')
    # scf.cf.param.set_value('posCtlPid.xKd', '1.0')
    # scf.cf.param.set_value('posCtlPid.yKp', '2.5')
    # scf.cf.param.set_value('posCtlPid.yKi', '0.2')
    # scf.cf.param.set_value('posCtlPid.yKd', '1.0')
    # scf.cf.param.set_value('posCtlPid.zKp', '5.0')
    # scf.cf.param.set_value('posCtlPid.zKi', '0.5')
    # scf.cf.param.set_value('posCtlPid.zKd', '1.0')
    # scf.cf.param.set_value('posCtlPid.rLimit', '20.0')
    # scf.cf.param.set_value('posCtlPid.pLimit', '20.0')
    # scf.cf.param.set_value('velCtlPid.vxKp', '8.0')
    # scf.cf.param.set_value('velCtlPid.vxKi', '0.0')
    # scf.cf.param.set_value('velCtlPid.vxKd', '0.0')
    # scf.cf.param.set_value('velCtlPid.vyKp', '10.0')
    # scf.cf.param.set_value('velCtlPid.vyKi', '0.0')
    # scf.cf.param.set_value('velCtlPid.vyKd', '0.0')
    # scf.cf.param.set_value('velCtlPid.vzKp', '12.5')
    # scf.cf.param.set_value('velCtlPid.vzKi', '0.0')
    # scf.cf.param.set_value('velCtlPid.vzKd', '0.0')
    # scf.cf.param.set_value('posCtlPid.xBodyVelMax', '3.0')
    # scf.cf.param.set_value('posCtlPid.yBodyVelMax', '3.0')
    # scf.cf.param.set_value('posCtlPid.zVelMax', '3.0')

    # # Double loop in-body control
    scf.cf.param.set_value('posCtlPid.singleLoop', '0')
    scf.cf.param.set_value('posVelFilt.posFiltEn', '0')
    scf.cf.param.set_value('posVelFilt.posFiltCut', '5.0')
    scf.cf.param.set_value('posVelFilt.velFiltEn', '1')
    scf.cf.param.set_value('posVelFilt.velFiltCut', '10.0')
    scf.cf.param.set_value('posCtlPid.thrustBase', hover_thrust)
    scf.cf.param.set_value('posCtlPid.thrustMin', '20000')
    scf.cf.param.set_value('posCtlPid.xKp', '6.0')
    scf.cf.param.set_value('posCtlPid.xKi', '0.0')
    scf.cf.param.set_value('posCtlPid.xKd', '0.0')
    scf.cf.param.set_value('posCtlPid.yKp', '3.5')
    scf.cf.param.set_value('posCtlPid.yKi', '0.0')
    scf.cf.param.set_value('posCtlPid.yKd', '0.0')
    scf.cf.param.set_value('posCtlPid.zKp', '5.0')
    scf.cf.param.set_value('posCtlPid.zKi', '0.5')
    scf.cf.param.set_value('posCtlPid.zKd', '0.0')
    scf.cf.param.set_value('posCtlPid.rLimit', '25.0')
    scf.cf.param.set_value('posCtlPid.pLimit', '25.0')
    scf.cf.param.set_value('velCtlPid.vxKp', '6.0')
    scf.cf.param.set_value('velCtlPid.vxKi', '0.5')
    scf.cf.param.set_value('velCtlPid.vxKd', '0.0')
    scf.cf.param.set_value('velCtlPid.vyKp', '10.0')
    scf.cf.param.set_value('velCtlPid.vyKi', '0.5')
    scf.cf.param.set_value('velCtlPid.vyKd', '0.0')
    scf.cf.param.set_value('velCtlPid.vzKp', '12.5')
    scf.cf.param.set_value('velCtlPid.vzKi', '5.0')
    scf.cf.param.set_value('velCtlPid.vzKd', '0.0')
    scf.cf.param.set_value('posCtlPid.xBodyVelMax', '3.0')
    scf.cf.param.set_value('posCtlPid.yBodyVelMax', '3.0')
    scf.cf.param.set_value('posCtlPid.zVelMax', '3.0')

def set_control_parameters_Bolt(scf):
    print('Setting control parameters for Nimble Flapper Bolt')

    # scf.cf.param.set_value('pid_attitude.roll_kp', '15.0') #15
    # scf.cf.param.set_value('pid_attitude.roll_kd', '1.0') #1
    # scf.cf.param.set_value('pid_rate.roll_kp', '50.0') #50
    # scf.cf.param.set_value('pid_attitude.yaw_kp', '30.0')
    # scf.cf.param.set_value('pid_attitude.yaw_kd', '1.0')
    # scf.cf.param.set_value('pid_rate.yaw_kp', '80.0')

    # # Double loop in-body control
    # scf.cf.param.set_value('posCtlPid.thrustBase', hover_thrust)
    # scf.cf.param.set_value('posCtlPid.thrustMin', '20000')
    # scf.cf.param.set_value('posCtlPid.xKp', '4.0')
    # scf.cf.param.set_value('posCtlPid.xKi', '0.0')
    # scf.cf.param.set_value('posCtlPid.xKd', '0.0')
    # scf.cf.param.set_value('posCtlPid.yKp', '2.5')
    # scf.cf.param.set_value('posCtlPid.yKi', '0.0')
    # scf.cf.param.set_value('posCtlPid.yKd', '0.0')
    # scf.cf.param.set_value('posCtlPid.zKp', '5.0')
    # scf.cf.param.set_value('posCtlPid.zKi', '0.5')
    # scf.cf.param.set_value('posCtlPid.zKd', '0.0')
    # scf.cf.param.set_value('posCtlPid.rLimit', '25.0')
    # scf.cf.param.set_value('posCtlPid.pLimit', '25.0')
    # scf.cf.param.set_value('velCtlPid.vxKp', '4.0')
    # scf.cf.param.set_value('velCtlPid.vxKi', '0.5')
    # scf.cf.param.set_value('velCtlPid.vxKd', '0.0')
    # scf.cf.param.set_value('velCtlPid.vyKp', '10.0')
    # scf.cf.param.set_value('velCtlPid.vyKi', '0.5')
    # scf.cf.param.set_value('velCtlPid.vyKd', '0.0')
    # scf.cf.param.set_value('velCtlPid.vzKp', '12.5')
    # scf.cf.param.set_value('velCtlPid.vzKi', '5.0')
    # scf.cf.param.set_value('velCtlPid.vzKd', '0.0')
    # scf.cf.param.set_value('posCtlPid.xBodyVelMax', '3.0')
    # scf.cf.param.set_value('posCtlPid.yBodyVelMax', '3.0')
    # scf.cf.param.set_value('posCtlPid.zVelMax', '3.0')

    scf.cf.param.set_value('attFilt.rateFiltEn', '1') #1
    scf.cf.param.set_value('attFilt.omxFiltCut', '15') #12.5
    scf.cf.param.set_value('attFilt.omyFiltCut', '25') #12.5
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
    scf.cf.param.set_value('posCtlPid.xKp', '4.0')
    scf.cf.param.set_value('posCtlPid.xKi', '0.0')
    scf.cf.param.set_value('posCtlPid.xKd', '0.0')
    scf.cf.param.set_value('posCtlPid.yKp', '2.5')
    scf.cf.param.set_value('posCtlPid.yKi', '0.0')
    scf.cf.param.set_value('posCtlPid.yKd', '0.0')
    scf.cf.param.set_value('posCtlPid.zKp', '5.0')
    scf.cf.param.set_value('posCtlPid.zKi', '0.5')
    scf.cf.param.set_value('posCtlPid.zKd', '0.0')
    scf.cf.param.set_value('posCtlPid.rLimit', '25.0')
    scf.cf.param.set_value('posCtlPid.pLimit', '25.0')
    scf.cf.param.set_value('velCtlPid.vxKFF', '0.0')
    scf.cf.param.set_value('velCtlPid.vxKp', '4.0')
    scf.cf.param.set_value('velCtlPid.vxKi', '0.5')
    scf.cf.param.set_value('velCtlPid.vxKd', '0.0')
    scf.cf.param.set_value('velCtlPid.vyKFF', '0.0')
    scf.cf.param.set_value('velCtlPid.vyKp', '10.0')
    scf.cf.param.set_value('velCtlPid.vyKi', '0.5')
    scf.cf.param.set_value('velCtlPid.vyKd', '0.0')
    scf.cf.param.set_value('velCtlPid.vzKp', '12.5')
    scf.cf.param.set_value('velCtlPid.vzKi', '5.0')
    scf.cf.param.set_value('velCtlPid.vzKd', '0.0')
    scf.cf.param.set_value('posCtlPid.xBodyVelMax', '3.0')
    scf.cf.param.set_value('posCtlPid.yBodyVelMax', '3.0')
    scf.cf.param.set_value('posCtlPid.zVelMax', '3.0')

def set_control_parameters_CF(scf):
    print('Setting control parameters for Crazyflie')
    scf.cf.param.set_value('posCtlPid.rpLimit', '30.0')
    scf.cf.param.set_value('posCtlPid.xyVelMax', '3.0')
    scf.cf.param.set_value('posCtlPid.zVelMax', '3.0')

def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def reset_tumble_detector(scf):
    cf = scf.cf
    cf.param.set_value('stabilizer.stop', '0')
    time.sleep(0.1)

def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf

    for position in sequence:
        print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for i in range(50):
        #cf.high_level_commander.go_to(x, y, z, yaw, 3)
            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)


    #cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    #time.sleep(0.1)


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


def position_control_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    position_x = data["stateEstimate.x"]
    position_y = data["stateEstimate.y"]
    position_z = data["stateEstimate.z"]
    roll = data["stateEstimate.roll"]
    pitch = data["stateEstimate.pitch"]
    yaw = data["stateEstimate.yaw"]
    POSITION_X.append(position_x)
    POSITION_Y.append(position_y)
    POSITION_Z.append(position_z)
    ROLL.append(roll)
    PITCH.append(pitch)
    YAW.append(yaw)

def command_control_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    roll = data["controller.roll"]
    pitch = data["controller.pitch"]
    yaw = data["controller.yaw"]
    CMD_ROLL.append(roll)
    CMD_PITCH.append(pitch)
    CMD_YAW.append(yaw)

def command_pos_control_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    tar_x = data["posCtl.targetX"]
    tar_y = data["posCtl.targetY"]
    tar_z = data["posCtl.targetZ"]
    TARGET_X.append(tar_x)
    TARGET_Y.append(tar_y)
    TARGET_Z.append(tar_z)

def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_x = 0.0
    initial_y = 0.0
    initial_z = -0.3
    initial_yaw = 0  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    # define logging parameters
    log_state = LogConfig(name='stateEstimate', period_in_ms=50)
    log_state.add_variable('stateEstimate.x', 'float')
    log_state.add_variable('stateEstimate.y', 'float')
    log_state.add_variable('stateEstimate.z', 'float')
    log_state.add_variable('stateEstimate.roll', 'float')
    log_state.add_variable('stateEstimate.pitch', 'float')
    log_state.add_variable('stateEstimate.yaw', 'float')

    log_vbat = LogConfig(name='pm', period_in_ms=500)
    log_vbat.add_variable('pm.vbat', 'float')

    log_set = LogConfig(name='controller', period_in_ms=50)
    log_set.add_variable('controller.roll', 'float') # these are setpoints!!!
    log_set.add_variable('controller.pitch', 'float')
    log_set.add_variable('controller.yaw', 'float')

    #log_cmd = LogConfig(name='controller', period_in_ms=50)
    #log_cmd.add_variable('controller.cmd_thrust', 'float')
    #log_cmd.add_variable('controller.cmd_roll', 'float')
    #log_cmd.add_variable('controller.cmd_pitch', 'float')
    #log_cmd.add_variable('controller.cmd_yaw', 'float')

    #log_range1 = LogConfig(name='ranging', period_in_ms=50)
    #log_range1.add_variable('ranging.distance0', 'float')
    #log_range1.add_variable('ranging.distance1', 'float')
    #log_range1.add_variable('ranging.distance2', 'float')
    #log_range1.add_variable('ranging.distance3', 'float')

    #log_range2 = LogConfig(name='ranging', period_in_ms=50)
    #log_range2.add_variable('ranging.distance4', 'float')
    #log_range2.add_variable('ranging.distance5', 'float')
    #log_range2.add_variable('ranging.distance6', 'float')
    #log_range2.add_variable('ranging.distance7', 'float')

    log_pos_control = LogConfig(name='posCtl', period_in_ms=50)
    log_pos_control.add_variable('posCtl.targetX', 'float')
    log_pos_control.add_variable('posCtl.targetY', 'float')
    log_pos_control.add_variable('posCtl.targetZ', 'float')
    #log_pos_control.add_variable('posCtl.targetVX', 'float')
    #log_pos_control.add_variable('posCtl.targetVY', 'float')
    #log_pos_control.add_variable('posCtl.targetVZ', 'float')


    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with open('/home/guillermoga/flight_log_' + datetime.now().strftime("%Y%m%d-%H%M%S") + '.txt', 'w') as f:
            print('Flight log' + datetime.now().strftime("%Y%m%d %H%M%S"), file=f)

            #log_async_setup(scf, log_state)
            #log_async_setup(scf, log_set)
            #log_async_setup(scf, log_cmd)
            #log_async_setup(scf, log_range1)
            #log_async_setup(scf, log_range2)
            #log_async_setup(scf, log_pos_control)
            scf.cf.log.add_config(log_state)
            log_state.data_received_cb.add_callback(position_control_callback)
            scf.cf.log.add_config(log_pos_control)
            log_pos_control.data_received_cb.add_callback(command_pos_control_callback)
            scf.cf.log.add_config(log_set)
            log_set.data_received_cb.add_callback(command_control_callback)

            log_async_setup(scf, log_vbat)
            log_vbat.start()

            cf = scf.cf

            time.sleep(1)
            print('Battery voltage is {}' .format(vbat))

            reset_tumble_detector(scf)

            if (drone == 'Flapper'):
                set_control_parameters_Bolt(scf)
            elif (drone == 'FlapperRoadrunner'):
                set_control_parameters_Roadrunner(scf)
            elif (drone == 'CF'):
                set_control_parameters_CF(scf)

            # reinitialize_controller(scf) # to load all new control parameters

            set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
            reset_estimator(scf)

            log_state.start()
            log_set.start()
            #log_cmd.start()
            #log_range1.start()
            #log_range2.start()
            log_pos_control.start()

            # unlock the engines
            # cf.commander.send_setpoint(0, 0, 0, 0)

            activate_high_level_commander(cf)
            time.sleep(0.2)
            commander = cf.high_level_commander

            commander.takeoff(-0.5, 0.001) # setting the setpoint underground to spinup the motors before taking off
            time.sleep(0.01)
            commander.go_to(origin[0], origin[1], origin[2]-0.5, 0.0, 0.01) # (re)set the setpoint, sometimes xy stays at [0,0]
            time.sleep(0.2)
            commander.go_to(origin[0], origin[1], origin[2], 0.0, 1.5)
            time.sleep(0.5)

            for position in sequence:
                print('Setting position {}'.format(position))

                x = position[0] + initial_x
                y = position[1] + initial_y
                z = position[2] + initial_z

                commander.go_to(x, y, z, 0.0, 3)
                time.sleep(3.0)

            #commander.go_to(initial_x, initial_y, initial_z, 0.0, 2)
            #time.sleep(2.0)
            commander.land(-0.2, 3.0)
            time.sleep(3.0)
            commander.stop()

            log_state.stop()
            log_set.stop()
            #log_cmd.stop()
            log_vbat.stop()
            #log_range1.stop()
            #log_range2.stop()
            log_pos_control.stop()

            fig, axs = plt.subplots(3, 2)
            axs[0, 0].plot(POSITION_X)
            axs[0, 0].plot(TARGET_X)
            axs[0, 0].legend(["est", "cmd"])
            axs[0, 0].set_title("Position X")
            axs[1, 0].plot(POSITION_Y)
            axs[1, 0].plot(TARGET_Y)
            axs[1, 1].legend(["est", "cmd"])
            axs[1, 0].set_title("Position Y")
            axs[2, 0].plot(POSITION_Z)
            axs[2, 0].plot(TARGET_Z)
            axs[2, 0].legend(["est", "cmd"])
            axs[2, 0].set_title("Position Z")
            axs[0, 1].plot(PITCH)
            axs[0, 1].plot(CMD_PITCH)
            axs[0, 1].legend(["est", "cmd"])
            axs[0, 1].set_title("Pitch")
            axs[1, 1].plot(ROLL)
            axs[1, 1].plot(CMD_ROLL)
            axs[1, 1].legend(["est", "cmd"])
            axs[1, 1].set_title("Roll")
            axs[2, 1].plot(YAW)
            axs[2, 1].plot(CMD_YAW)
            axs[2, 1].legend(["est", "cmd"])
            axs[2, 1].set_title("Yaw")
            plt.show()
