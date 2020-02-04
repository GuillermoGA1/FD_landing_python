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
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

import threading as th

kill_flight = False
def key_capture_thread():
    global kill_flight
    input()
    kill_flight = True


# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'

# Change the sequence according to your setup
#             x    y    z
sequence = [
    (0, 0, 1.0),
    (0, 0, 0.6)
]

#sequence = [
#    (0, 0, 1.2),
#    (-0.7, 0, 1.2),
#    (0, -0.7, 1.2),
#    (0, 0, 1.2),
#    (0, 0, 1.2),
#    (0, 0, 1.2),
#    (0, 0, 1.2),
#    (0, 0, 1.2),
#    (0, 0, 1.2),
#    (0, 0, 0.25),
#]


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

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

def print_position():
    with SyncLogger(scf, lg_posEst) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = logger[2]

            print('[%d]: %s' % (timestamp, data))
            break

def print_position_setpoint():
    with SyncLogger(scf, lg_posSet) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = logger[2]

            print('[%d]: %s' % (timestamp, data))
            break


def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf
    
    # unlock the engines
    cf.commander.send_setpoint(0.0, 0.0, 0, 0)

    # start listening to Enter key
    th.Thread(target=key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()
    
    print('Starting engines (press enter to kill)')

    for i in range(10):
        if kill_flight:
            break
        
        print_position()

        cf.commander.send_setpoint(0.0, 0.0, 0, 20000)
        time.sleep(0.1)
        

    for position in sequence:
        
        print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for i in range(50):
            if kill_flight:
                cf.commander.send_stop_setpoint()
                print('Drone killed')
                return

            print_position()
            # print_position_setpoint()

            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_x = 0.0
    initial_y = 0.0
    initial_z = 0.0
    initial_yaw = 0  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    # define logging parameters
    lg_posEst = LogConfig(name='stateEstimate', period_in_ms=20)
    lg_posEst.add_variable('stateEstimate.x', 'float')
    lg_posEst.add_variable('stateEstimate.y', 'float')
    lg_posEst.add_variable('stateEstimate.z', 'float')

    # lg_posSet = LogConfig(name='posCtl', period_in_ms=20)
    # lg_posSet.add_variable('posCtl.targetX', 'float')
    # lg_posSet.add_variable('posCtl.targetY', 'float')
    # lg_posSet.add_variable('posCtl.targetZ', 'float')
    
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_tumble_detector(scf)
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        reset_estimator(scf)
        run_sequence(scf, sequence,
                     initial_x, initial_y, initial_z, initial_yaw)
