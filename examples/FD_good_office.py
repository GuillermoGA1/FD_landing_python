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


D0 = []
D1 = []
D2 = []
ROLL = []
PITCH = []
YAW = []
POSITION_X = []
POSITION_Y = []
POSITION_Z = []
VX = []
VY = []
VZ = []

TAR_X = []
TAR_Y = []
TAR_Z = []
TAR_VX = []
TAR_VY = []
TAR_VZ = []
TAR_ROLL = []
TAR_PITCH = []
TAR_YAW = []



# Change the sequence according to your setup
#             x    y    z

a = 1.5 #length of side
h = 1.2 #height
pi = 3.1416
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
    #(0, 0, h, 0),
    (0, 0, h, 0),
    (0, 0, h, 0),
    (a, 0, h, 0),
    (a, 0, h, 0),
    (a, 0, h, 0),
   # (a, a, h, 0),
   # (a, a, h, 0),
   # (0, a, h, 0),
   # (0, a, h, 0),
    (0, 0, h, 0),
    (0, 0, h, 0),
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

origin = [0, 0, 0.2]
initial_x = 0
initial_y = 0
initial_z = 0.2
initial_yaw = 0  # In degrees
landing_yaw = 0
# 0: positive X direction
# 90: positive Y direction
# 180: negative X direction
# 270: negative Y direction

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
    scf.cf.param.set_value('posVelFilt.posFiltCut', '5.0') #5
    scf.cf.param.set_value('posVelFilt.velFiltEn', '0')
    scf.cf.param.set_value('posVelFilt.velFiltCut', '10.0') #10
    scf.cf.param.set_value('posVelFilt.posZFiltEn', '0')
    scf.cf.param.set_value('posVelFilt.posZFiltCut', '5.0') #5
    scf.cf.param.set_value('posVelFilt.velZFiltEn', '1')    #1
    scf.cf.param.set_value('posVelFilt.velZFiltCut', '7.0') #10
    scf.cf.param.set_value('posCtlPid.thrustBase', hover_thrust)
    scf.cf.param.set_value('posCtlPid.thrustMin', '15000')

    #POSITION
    scf.cf.param.set_value('posCtlPid.xKp', '7.0') #4 6 test_good: 9.5
    scf.cf.param.set_value('posCtlPid.xKi', '0.0') #0 test_good: 0
    scf.cf.param.set_value('posCtlPid.xKd', '1.0') #0   test_good 1.0
    scf.cf.param.set_value('posCtlPid.yKp', '5.0') #2.5
    scf.cf.param.set_value('posCtlPid.yKi', '0.0') #0
    scf.cf.param.set_value('posCtlPid.yKd', '0.0') #0 
    scf.cf.param.set_value('posCtlPid.zKp', '5.0') #5 
    scf.cf.param.set_value('posCtlPid.zKi', '0.5') #0.5
    scf.cf.param.set_value('posCtlPid.zKd', '0.2') #0
    scf.cf.param.set_value('posCtlPid.rLimit', '45.0') #25
    scf.cf.param.set_value('posCtlPid.pLimit', '40.0') #25

    #VELOCITY
    scf.cf.param.set_value('velCtlPid.vxKFF', '2.0') #0 tested: 0.7
    scf.cf.param.set_value('velCtlPid.vxKp', '6.0') #4   tested: 6
    scf.cf.param.set_value('velCtlPid.vxKi', '1.5') #0.5 tested:0.8
    scf.cf.param.set_value('velCtlPid.vxKd', '0.1') #0 tested: 0.2
    scf.cf.param.set_value('velCtlPid.vyKFF', '2.0') #0
    scf.cf.param.set_value('velCtlPid.vyKp', '5.0') #4 tested:5
    scf.cf.param.set_value('velCtlPid.vyKi', '0.9') #0.5 tested: 0.9
    scf.cf.param.set_value('velCtlPid.vyKd', '0.2') #0  tested: 0.2
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
    scf.cf.param.set_value('pid_attitude.pitch_kd', '1.0')  #1    test:1 
    scf.cf.param.set_value('pid_attitude.roll_kp', '10.0')  #15   test:10
    scf.cf.param.set_value('pid_attitude.roll_ki', '1.0')   #0    test:1
    scf.cf.param.set_value('pid_attitude.roll_kd', '0.5')   #1    test:0.2
    scf.cf.param.set_value('pid_attitude.yaw_kp', '45.0')   #30   test:45
    scf.cf.param.set_value('pid_attitude.yaw_ki', '0')   #0    test:20
    scf.cf.param.set_value('pid_attitude.yaw_kd', '1.0')    #1    test:1

    #ATTITUDE RATES
    scf.cf.param.set_value('pid_rate.pitch_kp', '60.0')  #70
    scf.cf.param.set_value('pid_rate.pitch_ki', '0')  
    scf.cf.param.set_value('pid_rate.pitch_kd', '0') 
    scf.cf.param.set_value('pid_rate.roll_kp', '50.0')  #50
    scf.cf.param.set_value('pid_rate.roll_ki', '0')   
    scf.cf.param.set_value('pid_rate.roll_kd', '0')   
    scf.cf.param.set_value('pid_rate.yaw_kp', '100.0')   #80 
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
   # print('[%d]: %s' % (timestamp, data), file=f)
    if 'pm.vbat' in data:
        global vbat
        vbat = data['pm.vbat']
    #if 'stateEstimate.x' in data:
    #    data["timestamp"] = timestamp
    #    writer = csv.DictWriter(open('/home/guillermoga/flight_log.csv', 'a+', newline=''), fieldnames=["timestamp","stateEstimate.x","stateEstimate.y","stateEstimate.z"])
    #    writer.writerow(data)

def log_and_print_async_callback(timestamp, data, logconf):
   # print('[%d]: %s' % (timestamp, data), file=f)
    print('[%d]: %s' % (timestamp, data))


def attitude_estimate_callback(timestamp, data, logconf):
 #   print('[%d]: %s' % (timestamp, data), file=f)
    roll = data["stateEstimate.roll"]
    pitch = data["stateEstimate.pitch"]
    yaw = data["stateEstimate.yaw"]
    ROLL.append(roll)
    PITCH.append(pitch)
    YAW.append(yaw)

def position_estimate_callback(timestamp, data, logconf):
  #  print('[%d]: %s' % (timestamp, data), file=f)
    position_x = data["stateEstimate.x"]
    position_y = data["stateEstimate.y"]
    position_z = data["stateEstimate.z"]
    POSITION_X.append(position_x)
    POSITION_Y.append(position_y)
    POSITION_Z.append(position_z)

def velocity_estimate_callback(timestamp, data, logconf):
  #  print('[%d]: %s' % (timestamp, data), file=f)
    velocity_x = data["stateEstimate.vx"]
    velocity_y = data["stateEstimate.vy"]
    velocity_z = data["stateEstimate.vz"]
    VX.append(velocity_x)
    VY.append(velocity_y)
    VZ.append(velocity_z)

def position_control_callback(timestamp, data, logconf):
 #   print('[%d]: %s' % (timestamp, data), file=f)
    tar_x = data["posCtl.targetX"]
    tar_y = data["posCtl.targetY"]
    tar_z = data["posCtl.targetZ"]
    TAR_X.append(tar_x)
    TAR_Y.append(tar_y)
    TAR_Z.append(tar_z)

def velocity_control_callback(timestamp, data, logconf):
  #  print('[%d]: %s' % (timestamp, data), file=f)
    tar_vx = data["posCtl.targetVX"]
    tar_vy = data["posCtl.targetVY"]
    tar_vz = data["posCtl.targetVZ"]
    TAR_VX.append(tar_vx)
    TAR_VY.append(tar_vy)
    TAR_VZ.append(tar_vz)

def attitude_control_callback(timestamp, data, logconf):
 #   print('[%d]: %s' % (timestamp, data), file=f)
    roll = data["controller.roll"]
    pitch = data["controller.pitch"]
    yaw = data["controller.yaw"]
    TAR_ROLL.append(roll)
    TAR_PITCH.append(pitch)
    TAR_YAW.append(yaw)

def compl_estimate_callback(timestamp, data, logconf):
   # print('[%d]: %s' % (timestamp, data), file=f)
    roll = data["stateEstimate.roll_compl"]
    pitch = data["stateEstimate.pitch_compl"]
    yaw = data["stateEstimate.yaw_compl"]
    D0.append(roll)
    D1.append(pitch)
    D2.append(yaw)
'''
def quaternion_estimate_callback(timestamp, data, logconf):
  #  print('[%d]: %s' % (timestamp, data), file=f)
    qx = data["stateEstimate.qx"]
    qy = data["stateEstimate.qy"]
    qz = data["stateEstimate.qz"]
    qw = data["stateEstimate.qw"]
    QX.append(qx)
    QY.append(qy)
    QZ.append(qz)
    QW.append(qw)

def quaternionaux_estimate_callback(timestamp, data, logconf):
  #  print('[%d]: %s' % (timestamp, data), file=f)
    qx = data["stateEstimate.qx_aux"]
    qy = data["stateEstimate.qy_aux"]
    qz = data["stateEstimate.qz_aux"]
    qw = data["stateEstimate.qw_aux"]
    QX_A.append(qx)
    QY_A.append(qy)
    QZ_A.append(qz)
    QW_A.append(qw)

def quaternionmeasured_estimate_callback(timestamp, data, logconf):
  #  print('[%d]: %s' % (timestamp, data), file=f)
    qx = data["kalman.q1_aux"]
    qy = data["kalman.q2_aux"]
    qz = data["kalman.q3_aux"]
    qw = data["kalman.q0_aux"]
    QX_M.append(qx)
    QY_M.append(qy)
    QZ_M.append(qz)
    QW_M.append(qw)

def quaternionerror_estimate_callback(timestamp, data, logconf):
  #  print('[%d]: %s' % (timestamp, data), file=f)
    qx = data["kalman.q1"]
    qy = data["kalman.q2"]
    qz = data["kalman.q3"]
    qw = data["kalman.q0"]
    QX_E.append(qx)
    QY_E.append(qy)
    QZ_E.append(qz)
    QW_E.append(qw)
    '''


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor

    # define logging parameters

    log_vbat = LogConfig(name='pm', period_in_ms=500)
    log_vbat.add_variable('pm.vbat', 'float')

    log_pos_estimate = LogConfig(name='stateEstimate', period_in_ms=50)
    log_pos_estimate.add_variable('stateEstimate.x', 'float')
    log_pos_estimate.add_variable('stateEstimate.y', 'float')
    log_pos_estimate.add_variable('stateEstimate.z', 'float')
 
    log_att_estimate = LogConfig(name='stateEstimate', period_in_ms=50)
    log_att_estimate.add_variable('stateEstimate.roll', 'float')
    log_att_estimate.add_variable('stateEstimate.pitch', 'float')
    log_att_estimate.add_variable('stateEstimate.yaw', 'float')

    log_vel_estimate = LogConfig(name='stateEstimate', period_in_ms=50)
    log_vel_estimate.add_variable('stateEstimate.vx', 'float')
    log_vel_estimate.add_variable('stateEstimate.vy', 'float')
    log_vel_estimate.add_variable('stateEstimate.vz', 'float')

    log_pos_ctrl = LogConfig(name='posCtl', period_in_ms=50)
    log_pos_ctrl.add_variable('posCtl.targetX', 'float')
    log_pos_ctrl.add_variable('posCtl.targetY', 'float')
    log_pos_ctrl.add_variable('posCtl.targetZ', 'float')

    log_att_ctrl = LogConfig(name='controller', period_in_ms=50)
    log_att_ctrl.add_variable('controller.roll', 'float') # these are setpoints!!!
    log_att_ctrl.add_variable('controller.pitch', 'float')
    log_att_ctrl.add_variable('controller.yaw', 'float')

    log_vel_ctrl = LogConfig(name='posCtl', period_in_ms=50)
    log_vel_ctrl.add_variable('posCtl.targetVX', 'float')
    log_vel_ctrl.add_variable('posCtl.targetVY', 'float')
    log_vel_ctrl.add_variable('posCtl.targetVZ', 'float')

    log_compl_estimate = LogConfig(name='stateEstimate', period_in_ms=50)
    log_compl_estimate.add_variable('stateEstimate.roll_compl', 'float')
    log_compl_estimate.add_variable('stateEstimate.pitch_compl', 'float')
    log_compl_estimate.add_variable('stateEstimate.yaw_compl', 'float')

    '''
    log_att_estimate = LogConfig(name='stateEstimate', period_in_ms=50)
    log_att_estimate.add_variable('stateEstimate.roll', 'float')
    log_att_estimate.add_variable('stateEstimate.pitch', 'float')
    log_att_estimate.add_variable('stateEstimate.yaw', 'float')

    log_compl_estimate = LogConfig(name='stateEstimate', period_in_ms=50)
    log_compl_estimate.add_variable('stateEstimate.roll_compl', 'float')
    log_compl_estimate.add_variable('stateEstimate.pitch_compl', 'float')
    log_compl_estimate.add_variable('stateEstimate.yaw_compl', 'float')

    log_quaternion_estimate = LogConfig(name = "statEstimate", period_in_ms=50)
    log_quaternion_estimate.add_variable('stateEstimate.qx', 'float')
    log_quaternion_estimate.add_variable('stateEstimate.qy', 'float')
    log_quaternion_estimate.add_variable('stateEstimate.qz', 'float')
    log_quaternion_estimate.add_variable('stateEstimate.qw', 'float')

    log_quaternionaux_estimate = LogConfig(name = "statEstimate", period_in_ms=50)
    log_quaternionaux_estimate.add_variable('stateEstimate.qx_aux', 'float')
    log_quaternionaux_estimate.add_variable('stateEstimate.qy_aux', 'float')
    log_quaternionaux_estimate.add_variable('stateEstimate.qz_aux', 'float')
    log_quaternionaux_estimate.add_variable('stateEstimate.qw_aux', 'float')

    log_quaternionmeasured_estimate = LogConfig(name = "kalman", period_in_ms=50)
    log_quaternionmeasured_estimate.add_variable('kalman.q0_aux', 'float')
    log_quaternionmeasured_estimate.add_variable('kalman.q1_aux', 'float')
    log_quaternionmeasured_estimate.add_variable('kalman.q2_aux', 'float')
    log_quaternionmeasured_estimate.add_variable('kalman.q3_aux', 'float')

    log_quaternionerror_estimate = LogConfig(name = "kalman", period_in_ms=50)
    log_quaternionerror_estimate.add_variable('kalman.q0', 'float')
    log_quaternionerror_estimate.add_variable('kalman.q1', 'float')
    log_quaternionerror_estimate.add_variable('kalman.q2', 'float')
    log_quaternionerror_estimate.add_variable('kalman.q3', 'float')
    '''


    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
       # with open('/home/guillermoga/quaternions_static_' + datetime.now().strftime("%Y%m%d-%H%M%S") + '.txt', 'w') as f:
       #     print('Flight log' + datetime.now().strftime("%Y%m%d %H%M%S"), file=f)

            '''
            scf.cf.log.add_config(log_att_estimate)
            log_att_estimate.data_received_cb.add_callback(attitude_estimate_callback)
            scf.cf.log.add_config(log_compl_estimate)
            log_compl_estimate.data_received_cb.add_callback(compl_estimate_callback)
            scf.cf.log.add_config(log_quaternion_estimate)
            log_quaternion_estimate.data_received_cb.add_callback(quaternion_estimate_callback)
            scf.cf.log.add_config(log_quaternionaux_estimate)
            log_quaternionaux_estimate.data_received_cb.add_callback(quaternionaux_estimate_callback)
            scf.cf.log.add_config(log_quaternionmeasured_estimate)
            log_quaternionmeasured_estimate.data_received_cb.add_callback(quaternionmeasured_estimate_callback)
            scf.cf.log.add_config(log_quaternionerror_estimate)
            log_quaternionerror_estimate.data_received_cb.add_callback(quaternionerror_estimate_callback)
            '''

            scf.cf.log.add_config(log_pos_estimate)
            log_pos_estimate.data_received_cb.add_callback(position_estimate_callback)
            scf.cf.log.add_config(log_vel_estimate)
            log_vel_estimate.data_received_cb.add_callback(velocity_estimate_callback)
            scf.cf.log.add_config(log_att_estimate)
            log_att_estimate.data_received_cb.add_callback(attitude_estimate_callback)
            scf.cf.log.add_config(log_pos_ctrl)
            log_pos_ctrl.data_received_cb.add_callback(position_control_callback)
            scf.cf.log.add_config(log_vel_ctrl)
            log_vel_ctrl.data_received_cb.add_callback(velocity_control_callback)
            scf.cf.log.add_config(log_att_ctrl)
            log_att_ctrl.data_received_cb.add_callback(attitude_control_callback)
            scf.cf.log.add_config(log_compl_estimate)
            log_compl_estimate.data_received_cb.add_callback(compl_estimate_callback)

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
            log_pos_ctrl.start()
            log_vel_ctrl.start()
            log_att_ctrl.start()
            log_compl_estimate.start()

            '''
            log_quaternion_estimate.start()
            log_quaternionaux_estimate.start()
            log_quaternionmeasured_estimate.start()
            log_quaternionerror_estimate.start()
            '''
            print("Start log")

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
            
            
            #time.sleep(20)
            log_pos_estimate.stop()
            log_vel_estimate.stop()
            log_att_estimate.stop()
            log_pos_ctrl.stop()
            log_vel_ctrl.stop()
            log_att_ctrl.stop()
            log_compl_estimate.stop()

            '''
            log_att_estimate.stop()
            log_quaternion_estimate.stop()
            log_quaternionaux_estimate.stop()
            log_quaternionmeasured_estimate.stop()
            log_quaternionerror_estimate.stop()
            '''
            print("Finished log")


            fig, axs = plt.subplots(3, 3)
            axs[0, 0].plot(POSITION_X)
            axs[0, 0].plot(TAR_X)
            axs[0, 0].grid()
            axs[0, 0].legend(["est", "cmd"])
            axs[0, 0].set_title("Position X")
            axs[1, 0].plot(POSITION_Y )
            axs[1, 0].plot(TAR_Y)
            axs[1, 0].grid()
            axs[1, 0].legend(["est", "cmd"])
            axs[1, 0].set_title("Position Y")
            axs[2, 0].plot(POSITION_Z)
            axs[2, 0].plot(TAR_Z)
            axs[2, 0].grid()
            axs[2, 0].legend(["est", "cmd"])
            axs[2, 0].set_title("Position Z")
            axs[0, 1].plot(VX)
            axs[0, 1].plot(TAR_VX)
            axs[0, 1].grid()
            axs[0, 1].legend(["est", "cmd"])
            axs[0, 1].set_title("Velocity X")
            axs[1, 1].plot(VY)
            axs[1, 1].plot(TAR_VY)
            axs[1, 1].grid()
            axs[1, 1].legend(["est", "cmd"])
            axs[1, 1].set_title("Velocity Y")
            axs[2, 1].plot(VZ)
            axs[2, 1].plot(TAR_VZ)
            axs[2, 1].grid()
            axs[2, 1].legend(["est", "cmd"])
            axs[2, 1].set_title("Velocity Z")
            axs[0, 2].plot(PITCH)
            axs[0, 2].plot(TAR_PITCH)
            axs[0, 2].grid()
            axs[0, 2].legend(["est", "cmd"])
            axs[0, 2].set_title("Pitch")
            axs[1, 2].plot(ROLL)
            axs[1, 2].plot(TAR_ROLL)
            axs[1, 2].grid()
            axs[1, 2].legend(["est", "cmd"])
            axs[1, 2].set_title("Roll")
            axs[2, 2].plot(YAW)
            axs[2, 2].plot(TAR_YAW)
            axs[2, 2].grid()
            axs[2, 2].legend(["est", "cmd"])
            axs[2, 2].set_title("Yaw")

            fig2, axs2 = plt.subplots(3, 2)
            axs2[0, 0].plot(ROLL)
            axs2[0, 0].grid()
            axs2[0, 0].set_title("Roll Kalman")
            axs2[1, 0].plot(PITCH)
            axs2[1, 0].grid()
            axs2[1, 0].set_title("Pitch Kalman")
            axs2[2, 0].plot(YAW)
            axs2[2, 0].grid()
            axs2[2, 0].set_title("Yaw Kalman")
            axs2[0, 1].plot(D0)
            axs2[0, 1].grid()
            axs2[0, 1].set_title("Roll aux")
            axs2[1, 1].plot(D1)
            axs2[1, 1].grid()
            axs2[1, 1].set_title("Pitch aux")
            axs2[2, 1].plot(D2)
            axs2[2, 1].grid()
            axs2[2, 1].set_title("Yaw aux")
    

            '''
            fig2, axs2 = plt.subplots(2,2)
            axs2[0, 0].plot(QW)
            axs2[0, 0].plot(QW_A)
            axs2[0, 0].grid()
            axs2[0, 0].legend(["main", "aux"]) 
            axs2[0 ,0].set_title("QW")
            axs2[0, 1].plot(QX)
            axs2[0, 1].plot(QX_A)
            axs2[0, 1].grid()
            axs2[0, 1].legend(["main", "aux"]) 
            axs2[0 ,1].set_title("QX")  
            axs2[1, 0].plot(QY)
            axs2[1, 0].plot(QY_A)
            axs2[1, 0].grid()
            axs2[1, 0].legend(["main", "aux"]) 
            axs2[1 ,0].set_title("QY") 
            axs2[1, 1].plot(QZ)
            axs2[1, 1].plot(QZ_A)
            axs2[1, 1].grid()
            axs2[1, 1].legend(["main", "aux"]) 
            axs2[1 ,1].set_title("QZ")      

            
            #Measured for Kalman and estimate for kalman after error
            fig3, axs3 = plt.subplots(2,2)
            axs3[0, 0].plot(QW_M)
            axs3[0, 0].plot(QW_E)
            axs3[0, 0].grid()
            axs3[0, 0].legend(["measured", "error"]) 
            axs3[0 ,0].set_title("QW")
            axs3[0, 1].plot(QX_M)
            axs3[0, 1].plot(QX_E)
            axs3[0, 1].grid()
            axs3[0, 1].legend(["measured", "error"]) 
            axs3[0 ,1].set_title("QX")  
            axs3[1, 0].plot(QY_M)
            axs3[1, 0].plot(QY_E)
            axs3[1, 0].grid()
            axs3[1, 0].legend(["measured", "error"]) 
            axs3[1 ,0].set_title("QY") 
            axs3[1, 1].plot(QZ_M)
            axs3[1, 1].plot(QZ_E)
            axs3[1, 1].grid()
            axs3[1, 1].legend(["mesaured", "error"]) 
            axs3[1 ,1].set_title("QE")     
            '''
            plt.show()

       

