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
from datetime import datetime
import time

import threading as th

kill_flight = False
def key_capture_thread():
    global kill_flight
    input()
    kill_flight = True

#drone = 'CF'
drone = 'FlapperRoadrunner'
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

# Change the sequence according to your setup
#             x    y    z   yaw[deg]
sequence = [
    (0, 0.25, 1.25, 0.0),
    (0, 0.25, 1.75, 0.0),
    (0, 0.25, 1.75, 0.0),
    (-0.5, 0.25, 1.75, 0.0),
    (-0.5, 0.25, 1.75, 0.0),
    (-0.5, 0.25, 1.75, 0.0),
    (0.5, 0.25, 1.75, 0.0),
    (0.5, 0.25, 1.75, 0.0),
    (0.5, 0.25, 1.75, 0.0),
    (0, 0.25, 1.75, 0.0),
    (0, 0.25, 1.75, 0.0),
    (0, 0.25, 1.75, 0.0),
    (0, 1.25, 1.75, 0.0),
    (0, 1.25, 1.75, 0.0),
    (0, 1.25, 1.75, 0.0),
    (0, 0.25, 1.75, 0.0),
    (0, 0.25, 1.75, 0.0),
    (0, 0.25, 1.75, 0.0),
    (0, 0.25, 1.75, 90.0),
    (0, 0.25, 1.75, 90.0),
    (0, 0.25, 1.75, 180.0),
    (0, 0.25, 1.75, 180.0),
    (0, 0.25, 1.75, 270.0),
    (0, 0.25, 1.75, 270.0),
    (0, 0.25, 1.75, 0.0),
    (0, 0.25, 1.3, 0.0),
    (0, 0.25, 1.0, 0.0),
    (0, 0.25, 0.7, 0.0),
    (0, 0.25, 0.5, 0.0),
]

# sequence = [
#     (0, 0.25, 1.25, 0.0),
#     (0, 0.25, 1.5, 0.0),
#     (0, 0.25, 1.0, 0.0),
#     (0, 0.25, 1.0, 180.0),
#     (0, 0.25, 1.0, 0.0),
#     (0, 0.25, 0.65, 0.0),
# ]

# sequence = [
#     (0, 0.25, 1.25, 0.0),
#     (0, 0.25, 1.5, 0.0),
#     (0, 0.25, 1.5, 0.0),
#     (0, 0.25, 1.5, 0.0),
#     (0, 0.25, 1.5, 0.0),
#     (0, 0.25, 1.5, 0.0),
#     (0, 0.25, 1.0, 0.0),
#     (0, 0.25, 0.5, 0.0),
# ]

# Setting for Nimble Flapper
startup_thrust=15000
hover_thrust = 40000
takeoff_thrust = int(1.2*hover_thrust)
landing_thrust = 32000

# # Setting for CF2.1
# startup_thrust=10000
# hover_thrust = 35000
# takeoff_thrust = int(1.2*hover_thrust)
# landing_thrust = 20000



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


def set_control_parameters_Roadrunner(scf):
    print('Setting control parameters for Nimble Flapper Roadrunner')

    # scf.cf.param.set_value('pid_attitude.roll_kp', '15.0') #15
    # scf.cf.param.set_value('pid_attitude.roll_kd', '1.0') #1
    # scf.cf.param.set_value('pid_rate.roll_kp', '50.0') #50
    # scf.cf.param.set_value('pid_attitude.yaw_kp', '30.0')
    # scf.cf.param.set_value('pid_attitude.yaw_kd', '1.0')
    # scf.cf.param.set_value('pid_rate.yaw_kp', '100.0')
    
    # # Single loop in-body control
    # scf.cf.param.set_value('posCtlPid.thrustBase', hover_thrust)
    # scf.cf.param.set_value('posCtlPid.thrustMin', '30000')
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
    # scf.cf.param.set_value('pid_attitude.yaw_kp', '30.0') #30
    # scf.cf.param.set_value('pid_attitude.yaw_kd', '1.0') #1
    # scf.cf.param.set_value('pid_rate.yaw_kp', '100.0') #100
    
    # Double loop in-body control - initial parameters
    # scf.cf.param.set_value('posCtlPid.thrustBase', hover_thrust)
    # scf.cf.param.set_value('posCtlPid.thrustMin', '30000')
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

    # Double loop in-body control
    scf.cf.param.set_value('posCtlPid.thrustBase', hover_thrust)
    scf.cf.param.set_value('posCtlPid.thrustMin', '30000')
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
    scf.cf.param.set_value('velCtlPid.vxKp', '8.0')
    scf.cf.param.set_value('velCtlPid.vxKi', '0.5')
    scf.cf.param.set_value('velCtlPid.vxKd', '0.0')
    scf.cf.param.set_value('velCtlPid.vyKp', '12.0')
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
    scf.cf.param.set_value('pid_attitude.yaw_kp', '30.0')
    scf.cf.param.set_value('pid_attitude.yaw_kd', '1.0')
    scf.cf.param.set_value('pid_rate.yaw_kp', '80.0')
    
    # Double loop in-body control
    scf.cf.param.set_value('posCtlPid.thrustBase', hover_thrust)
    scf.cf.param.set_value('posCtlPid.thrustMin', '30000')
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
    scf.cf.param.set_value('velCtlPid.vxKp', '4.0')
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
            
def log_and_print_async_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    print('[%d]: %s' % (timestamp, data))

def run_sequence(scf, sequence, base_x, base_y, base_z, f):
    cf = scf.cf
    
    ## TODO 
    # - rewrite as a state-machine
    # - check for vbat and if too low land - implement as an exception callback?

    # unlock the engines
    cf.commander.send_setpoint(0.0, 0.0, 0, 0)

    # start listening to Enter key
    th.Thread(target=key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()
    
    print('Starting engines (press enter to kill)')

    time0 = time.time()
    while (time.time()-time0) < 0.5:
        if kill_flight:
            cf.commander.send_stop_setpoint()
            print('Drone killed')
            break
        
        cf.commander.send_setpoint(0.0, 0.0, 0, startup_thrust)
        time.sleep(0.1)
    
    print('Taking off (press enter to kill)')
    time0 = time.time()
    while (time.time()-time0) < 0.5:
        if kill_flight:
            cf.commander.send_stop_setpoint()
            print('Drone killed')
            break
        
        cf.commander.send_setpoint(0.0, 0.0, 0, takeoff_thrust)
        time.sleep(0.1)
        

    for position in sequence:
        
        print('Setting position {}'.format(position), file=f)
        print('Setting position {}'.format(position))
        print('Battery voltage is {}' .format(vbat))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z
        yaw = position[3]

        time0 = time.time()
        while (time.time()-time0) < 3.0:
            if kill_flight:
                cf.commander.send_setpoint(0.0, 0.0, 0.0, landing_thrust)
                print('Drone forced to land')
                time.sleep(2.5)
                cf.commander.send_stop_setpoint()
                print('Drone killed')
                return

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
    log_state = LogConfig(name='stateEstimate', period_in_ms=20)
    log_state.add_variable('stateEstimate.x', 'float')
    log_state.add_variable('stateEstimate.y', 'float')
    log_state.add_variable('stateEstimate.z', 'float')
    log_state.add_variable('stateEstimate.roll', 'float')
    log_state.add_variable('stateEstimate.pitch', 'float')
    log_state.add_variable('stateEstimate.yaw', 'float')

    log_vbat = LogConfig(name='pm', period_in_ms=500)
    log_vbat.add_variable('pm.vbat', 'float')
    
    log_set = LogConfig(name='controller', period_in_ms=20)
    log_set.add_variable('controller.roll', 'float') # these are setpoints!!!
    log_set.add_variable('controller.pitch', 'float')
    log_set.add_variable('controller.yaw', 'float')
    
    log_cmd = LogConfig(name='controller', period_in_ms=20)
    log_cmd.add_variable('controller.cmd_thrust', 'float')
    log_cmd.add_variable('controller.cmd_roll', 'float')
    log_cmd.add_variable('controller.cmd_pitch', 'float')
    log_cmd.add_variable('controller.cmd_yaw', 'float')
    
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with open('/home/matej/bitcraze/FlightLogs/flight_log_' + datetime.now().strftime("%Y%m%d-%H%M%S") + '.txt', 'w') as f:
            print('Flight log' + datetime.now().strftime("%Y%m%d %H%M%S"), file=f)
            
            log_async_setup(scf, log_state)
            log_async_setup(scf, log_set)
            log_async_setup(scf, log_cmd)
            log_async_setup(scf, log_vbat)

            log_vbat.start()
            
            time.sleep(1)
            print('Battery voltage is {}' .format(vbat))
            
            reset_tumble_detector(scf)
            if (drone == 'Flapper'):
                set_control_parameters_Bolt(scf)
            elif (drone == 'FlapperRoadrunner'):
                set_control_parameters_Roadrunner(scf)
            set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
            reset_estimator(scf)
            
            log_state.start()
            log_set.start()
            log_cmd.start()

            run_sequence(scf, sequence, initial_x, initial_y, initial_z, f)

            log_state.stop()
            log_set.stop()
            log_cmd.stop()
            log_vbat.stop()
            