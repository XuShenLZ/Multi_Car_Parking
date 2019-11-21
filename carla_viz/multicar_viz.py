#!/usr/bin/env python

# Copyright (c) 2019 Intel Labs
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control with steering wheel Logitech G27.

To drive start by preshing the brake pedal.
Change your wheel_config.ini according to your steering wheel.

To find out the values of your steering wheel use jstest-gtk in Ubuntu.

"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
from utils.VehicleSpawner import VehicleSpawner, VehicleSpawnerByPose
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla
import pdb
from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import numpy as np
import random

# Calibrate the resistence of wheeel
import evdev
from evdev import ecodes, InputDevice

# Rosbag record
import subprocess, shlex
import time

from utils.carla_utils import *
if sys.version_info >= (3, 0):
    from configparser import ConfigParser
else:
    from ConfigParser import RawConfigParser as ConfigParser
try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')
try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# Fleet Management
import pickle
from coord_trans import Fleet
import pdb


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    drone_camera = None

    rosbag_proc = None

    trail = args.trail

    spwnr = None
    steady_spwnr = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        # client.load_world('exp')
        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args.filter)
        # print('Here')
        controller = DualControl(world, args.autopilot)

        # # "Drone" camera view
        blueprint_library = world.world.get_blueprint_library()
        drone_camera_bp = blueprint_library.find('sensor.camera.rgb')
        # drone_camera_bp.set_attribute('image_size_x', str(args.width))
        # drone_camera_bp.set_attribute('image_size_y', str(args.height))
        drone_camera_bp.set_attribute('image_size_x', str(600))
        drone_camera_bp.set_attribute('image_size_y', str(800))
        drone_camera_bp.set_attribute('fov', '100')
        drone_camera_bp.set_attribute('sensor_tick', '0.1')
        drone_camera_transform = carla.Transform(carla.Location(x=285.0, y=-210.0, z=20.0), carla.Rotation(yaw=90.0, pitch=-90))
        drone_camera = world.world.spawn_actor(drone_camera_bp, drone_camera_transform)

        np.random.seed(0)
        random.seed(0)

        # spwnr = None

        if args.record:
            now = datetime.datetime.now()
            now = now.strftime("%Y-%m-%d-%H-%M-%S")
            # client.start_recorder("/home/carla/PythonAPI/examples/bags/parking_p%s_s%d_e%d_%s.log" % (args.s_id, trail, ep, now))
            client.start_recorder("multicar_parking_%s.log" % now)

        # =============== Offset for coord transformation =======
        offset = [281.71-5.5, -239.9-(-33), np.pi/2 - 0]


        # ============== Steady vehicles =================
        with open('steady_vehicles.pickle', 'rb') as f:
            steady_v_locations = pickle.load(f)
        
        steady_spwnr_points = []
        for p in steady_v_locations:
            new_x   = p[1] + offset[0]
            new_y   = p[0] + offset[1]
            new_psi = random.choice([0, 180])

            steady_spwnr_points.append([new_x, new_y, new_psi])

        with open('trajectory.pickle', 'rb') as f:
            old_time, states, inputs = pickle.load(f)

        # =============== Fleet Establishment ===============
        # New time interval
        dt = 0.04
        new_time = np.arange(old_time[0], old_time[-1], dt)

        fleet = Fleet(old_time, states, inputs)

        fleet.transform_forall(offset)

        fleet.interp_forall(new_time)

        spwnr_points = []

        for car in fleet.vehicle_list:
            spwnr_points.append([car.interp_x[0], car.interp_y[0], car.interp_psi[0] / np.pi * 180])

        # print(spwnr_points)
        # =============== Spawn Vehicles =====================
        print(steady_spwnr_points)

        steady_spwnr = VehicleSpawnerByPose(client,False,steady_spwnr_points)
        spwnr = VehicleSpawnerByPose(client,False,spwnr_points)
        extra_spwnr = VehicleSpawnerByPose(client,False,[[281.71, -223.4, 180]])

        time.sleep(2)

        # =============== Traverse the list =================
        clock = pygame.time.Clock()
        for t in range(len(new_time)):
            for v_id in range(len(spwnr.vehicles_list)):
                vehicle = client.get_world().get_actor(spwnr.vehicles_list[v_id])
                transform = vehicle.get_transform()
                control   = vehicle.get_control()

                transform.location.x   = fleet.vehicle_list[v_id].interp_x[t]
                transform.location.y   = fleet.vehicle_list[v_id].interp_y[t]
                transform.rotation.yaw = fleet.vehicle_list[v_id].interp_psi[t] / np.pi * 180
                transform.rotation.roll = 0
                transform.rotation.pitch = 0

                control.steer = - fleet.vehicle_list[v_id].interp_steer[t] / np.pi * 180. / 35.
                control.gear  = -1

                if fleet.vehicle_list[v_id].interp_v[t] > 4.0:
                    control.brake = 0.0
                    control.throttle = 0.3
                else:
                    control.brake = 1.0
                    control.throttle = 0.0

                yaw = transform.rotation.yaw
                if (-185 < yaw and yaw < -175) or (-5 < yaw and yaw < 5) or (175 < yaw and yaw < 185):
                    control.steer = 0.0
                    # pass

                vehicle.apply_control(control)
                vehicle.set_transform(transform)



            world.tick(clock)
            world.render(display)
            pygame.display.flip()

            time.sleep(0.01)


        # var = spwnr.vehicles_list[0]
        # # print('var' ,var)
        # # print('dir', dir(var))
        # actor = client.get_world().get_actor(var)
        # location = actor.get_location()
        # location.x += 10
        # actor.set_location(location)


        # while True:
            # clock.tick_busy_loop(60)

        # controller_return = controller.parse_events(world, clock)

        # if controller_return == 10:
        #     print("Terminated by user")
        #     raise KeyboardInterrupt

        # if controller_return == 11:
        #     # Next episode
        #     break

        # if controller_return == 6:
        #     # publish intention
        #     pub_str = "Intention Determined"
        #     intention_pub.publish(pub_str)


    except Exception as e:
        print('got an exception', e)

    finally:
        if args.record:
            client.stop_recorder()
        if drone_camera:
            drone_camera.destroy()
        if world is not None:

            time.sleep(5)

            steady_spwnr.remove()
            spwnr.remove()
            extra_spwnr.remove()
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1920x1280',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.tesla.*',
        help='actor filter (default: "vehicle.*")')

    argparser.add_argument(
        '-i', '--s_id', 
        help="id of the subject",
        required=True,
        type=int)

    argparser.add_argument(
        '-t', '--trail', 
        help="trail number",
        required=True,
        type=int)

    argparser.add_argument(
        '-r', '--record', 
        help="Record rosbag and log",
        default=0,
        type=int)

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    # Calibrate the steer wheel
    device = evdev.list_devices()[0]
    evtdev = InputDevice(device)
    val = 20000
    evtdev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, val)

    print(__doc__)

    try:
        game_loop(args)
        

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
