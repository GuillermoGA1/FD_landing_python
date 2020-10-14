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
Simple example of a swarm using the High level commander.

The swarm takes off and flies a synchronous square shape before landing.
The trajectories are relative to the starting positions and the Crazyflies can
be at any position on the floor when the script is started.

This example is intended to work with any absolute positioning system.
It aims at documenting how to use the High Level Commander together with
the Swarm class.
"""
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger


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


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(scf)

def set_control_parameters(scf, drone):

    if (drone == 'Flapper'):
        print('Setting control parameters for FlapperBolt...')
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
        scf.cf.param.set_value('posCtlPid.thrustBase', '38000')
        scf.cf.param.set_value('posCtlPid.thrustMin', '15000')
        scf.cf.param.set_value('posCtlPid.xKp', '4.0')
        scf.cf.param.set_value('posCtlPid.xKi', '0.0')
        scf.cf.param.set_value('posCtlPid.xKd', '0.0')
        scf.cf.param.set_value('posCtlPid.yKp', '2.5')
        scf.cf.param.set_value('posCtlPid.yKi', '0.0')
        scf.cf.param.set_value('posCtlPid.yKd', '0.0')
        scf.cf.param.set_value('posCtlPid.zKp', '3.0')
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
    elif (drone == 'CF2'):
        print('Setting control parameters for CF2...')
        scf.cf.param.set_value('posCtlPid.rpLimit', '30.0')
        scf.cf.param.set_value('posCtlPid.xyVelMax', '3.0')
        scf.cf.param.set_value('posCtlPid.zVelMax', '3.0')

def reset_tumble_detector(scf):
    cf = scf.cf
    cf.param.set_value('stabilizer.stop', '0')
    time.sleep(0.1)

def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')

def set_initial_position(scf, x, y, z):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = 0.0
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def run_shared_sequence(scf, x0, y0, z0):
    
    commander = scf.cf.high_level_commander

    # commander.takeoff(1.25, 3.0)
    # time.sleep(4)
    # commander.go_to(0.0, 0, 0, 0, 1, relative=True)
    # time.sleep(4)
    # commander.go_to(0.0, 0, 0, 0, 1, relative=True)
    # time.sleep(4)
    # commander.go_to(0.5, 0, 0, 0, 1, relative=True)
    # time.sleep(4)
    # commander.go_to(-0.0, 0, 0, 0, 1, relative=True)
    # time.sleep(4)
    # commander.land(-0.7, 3.0)
    # time.sleep(2)
    # commander.stop()

    commander.takeoff(-0.5, 0.001) # setting the setpoint underground to spinup the motors before taking off
    time.sleep(0.01)
    commander.go_to(x0, y0, z0-0.5, 0.0, 0.01) # (re)set the setpoint, sometimes xy stays at [0,0]
    time.sleep(0.2)
    commander.go_to(x0, y0, z0+1.25, 0.0, 4)
    time.sleep(4.0)
    commander.go_to(x0, y0, z0+1.25, 0.0, 2)
    time.sleep(4.0)
    # commander.go_to(x0-0.5, y0, z0+1.25, 0.0, 1)
    # time.sleep(4.0)
    # commander.go_to(x0, y0, z0+1.25, 0.0, 1)
    # time.sleep(4.0)
    # commander.go_to(x0, y0-0.5, z0+1.25, 0.0, 1)
    # time.sleep(4.0)
    # commander.go_to(x0, y0+0.5, z0+1.25, 0.0, 1)
    # time.sleep(4.0)
    commander.go_to(x0, y0, z0+1.25, 0.0, 1)
    time.sleep(4.0)
    commander.land(-0.5, 3.0)
    time.sleep(3.0)
    commander.stop()


URI1 = 'radio://0/80/2M/E7E7E7E7E1' # FlapperBolt
URI2 = 'radio://0/80/2M/E7E7E7E7E7' # CF2
URI3 = 'radio://0/80/2M/E7E7E7E7E0' # FlapperRoadrunner

uris = {
    URI1,
    # URI2,
    URI3,
    # Add more URIs if you want more copters in the swarm
}

drones = {
            URI1: ['Flapper'],
            # URI2: ['CF2'],
            URI3: ['Flapper'],
        }

origin = [1.75, 2.25, 0]

origins = {
            URI1: [origin[0]-0.5, origin[1]-0.25, origin[2]],
            # URI2: [origin[0], origin[1], origin[2]],
            URI3: [origin[0]+0.5, origin[1]+0.25, origin[2]],
        }

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(reset_tumble_detector)
        swarm.parallel_safe(set_control_parameters, drones)
        swarm.parallel_safe(activate_high_level_commander)
        swarm.parallel_safe(set_initial_position, origins)
        swarm.parallel_safe(reset_estimator)
        swarm.parallel_safe(run_shared_sequence, origins)
