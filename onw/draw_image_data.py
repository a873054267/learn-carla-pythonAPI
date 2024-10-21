#!/usr/bin/env python
# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import math
import argparse
import copy
import time
from multiprocessing import Pool
from PIL import Image



import carla
import random

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """
    #sensors可传入camera，也可以传入lidar
    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self
    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data
    # ---------------

def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def get_image_as_array(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    # make the array writeable doing a deep copy
    array2 = copy.deepcopy(array)
    return array2

def draw_image(surface, array, blend=False):
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)

def get_screen_points(camera, K, image_w, image_h, points3d):
    
    # get 4x4 matrix to transform points from world to camera coordinates
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

    # build the points array in numpy format as (x, y, z, 1) to be operable with a 4x4 matrix
    points_temp = []
    for p in points3d:
        points_temp += [p.x, p.y, p.z, 1]
    points = np.array(points_temp).reshape(-1, 4).T
    
    # convert world points to camera space
    points_camera = np.dot(world_2_camera, points)
    
    # New we must change from UE4's coordinate system to an "standard"
    # (x, y ,z) -> (y, -z, x)
    # and we remove the fourth component also
    points = np.array([
        points_camera[1],
        points_camera[2] * -1,
        points_camera[0]])
    
    # Finally we can use our K matrix to do the actual 3D -> 2D.
    points_2d = np.dot(K, points)

    # normalize the values and transpose
    points_2d = np.array([
        points_2d[0, :] / points_2d[2, :],
        points_2d[1, :] / points_2d[2, :],
        points_2d[2, :]]).T

    return points_2d


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False

def write_image(frame, id, buffer):
    # Save the image using Pillow module.
    img = Image.fromarray(buffer)
    img.save('out/%s_%06d.png' % (id, frame))

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '--fov',
        default=60,
        type=int,
        help='FOV for camera')
    argparser.add_argument(
      '--res',
      metavar='WIDTHxHEIGHT',
      # default='1920x1080',
      default='800x600',
      help='window resolution (default: 800x600)')
    args = argparser.parse_args()
    
    args.width, args.height = [int(x) for x in args.res.split('x')]

    actor_list = []
    pygame.init()

    display = pygame.display.set_mode(
        (args.width, args.height),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    world = client.get_world()

    # spawn a camera 
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute("image_size_x", str(args.width))
    camera_bp.set_attribute("image_size_y", str(args.height))
    camera_bp.set_attribute("fov", str(args.fov))
    camera = world.spawn_actor(camera_bp, carla.Transform())
    
    # spawn a pedestrian
    world.set_pedestrians_seed(1235)
    ped_bp = random.choice(world.get_blueprint_library().filter("walker.pedestrian.*"))
    trans = carla.Transform()
    trans.location = world.get_random_location_from_navigation()
    ped = world.spawn_actor(ped_bp, trans)
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')

    controller = world.spawn_actor(walker_controller_bp, carla.Transform(), ped)
    controller.start()
    controller.go_to_location(world.get_random_location_from_navigation())
    controller.set_max_speed(1.5)

    # keep tracking of actors to remove
    actor_list.append(camera)
    actor_list.append(ped)
    actor_list.append(controller)


    try:
        #多线程用于图像的写出
        pool = Pool(processes=5)
        # Create a synchronous mode context.
        with CarlaSyncMode(world, camera, fps=30) as sync_mode:

            blending = 0
            turning = 0
            while True:
                if should_quit():
                    return
                clock.tick()

                # make some transition from custom pose to animation
                ped.blend_pose(math.sin(blending))
                # move the pedestrian
                blending += 0.015
                turning += 0.009
                # move camera around，设置相机跟随人
                trans = ped.get_transform()
                x = math.cos(turning) * -3
                y = math.sin(turning) * 3
                trans.location.x += x
                trans.location.y += y
                trans.location.z = 2
                trans.rotation.pitch = -16
                trans.rotation.roll = 0
                trans.rotation.yaw = -360 * (turning/(math.pi*2))
                camera.set_transform(trans)

                # Advance the simulation and wait for the data.
                snapshot, image_rgb = sync_mode.tick(timeout=5.0)
                buffer = get_image_as_array(image_rgb)

                draw_image(display, buffer)
                # pool.apply_async(write_image, (snapshot.frame, "ped", buffer))

                pygame.display.flip()

    finally:
        # time.sleep(5)
        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()
        pygame.quit()
        pool.close()
        print('done.')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
