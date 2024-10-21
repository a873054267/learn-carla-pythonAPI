import glob
import os
import sys

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================

import carla

import random
import time
import numpy as np
import queue
import logging
import math

actor_list = []
vehicles_id_list = []
set_synchronous = True  # 同步模式
tm_port = 8000  # tm端口

try:

    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    tm = client.get_trafficmanager(tm_port)

    # 设置车辆直接停车距离为3m
    tm.set_global_distance_to_leading_vehicle(0.1)

    # 设置车辆速度限速的百分之30
    tm.global_percentage_speed_difference(30.0)

    if set_synchronous:
        print('Pay attention this client set mode to synchronous')
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        # 需要同时为TM开启同步
        tm.set_synchronous_mode(True)

    blueprint_library = world.get_blueprint_library()

    bp = blueprint_library.find('vehicle.audi.a2')

    # 必须将测试车辆的role name设置为hero 才开启了混合物理模式
    bp.set_attribute('role_name', 'hero')

    spawn_point = random.choice(world.get_map().get_spawn_points())
    print('my vehicle blueprint:', bp, 'my car position:', spawn_point)
    vehicle = world.spawn_actor(bp, spawn_point)

    actor_list.append(vehicle)

    # 设置混合物理模式开启仅在测试车辆附近
    tm.set_hybrid_physics_mode(True)

    #  默认是70，这里设置了半径50m内开启
    tm.set_hybrid_physics_radius(50.0)

    # ------------- 此部分来源于spawn_npc.py ---------------------------- #
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    SetVehicleLightState = carla.command.SetVehicleLightState
    FutureActor = carla.command.FutureActor

    blueprints_vehicle = blueprint_library.filter("vehicle.*")
    blueprints = sorted(blueprints_vehicle, key=lambda bp: bp.id)
    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)
    # --------------
    # Spawn vehicles
    # --------------

    batch = []
    number_of_vehicles = 100
    for n, transform in enumerate(spawn_points):
        if n >= number_of_vehicles:
            break
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')

        # spawn the cars and set their autopilot and light state all together
        batch.append(SpawnActor(blueprint, transform)
                     .then(SetAutopilot(FutureActor, True, tm.get_port())))

    for response in client.apply_batch_sync(batch, set_synchronous):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_id_list.append(response.actor_id)
    # ------------- 此部分来源于spawn_npc.py ---------------------------- #

    # 获得所有车辆
    vehicles_list = world.get_actors().filter('vehicle.*')
    for v in vehicles_list:
        if v.attributes['role_name'] != "hero":
            # 设置可切车道
            tm.auto_lane_change(v, True)

    vehicle.set_autopilot(True)
    spectator = world.get_spectator()
    debugHelper =world.debug

    while True:
        if set_synchronous:
            world.tick()
        else:
            world.wait_for_tick()

        transform = vehicle.get_transform()
        debugHelper.draw_line(carla.Location(0,0,0),carla.Location(100,100,0))
        # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
        #                                         carla.Rotation(yaw=int(transform.rotation.yaw), roll=0, pitch=-90)))

        # 打印距离测试车辆10m以内所有车辆的速度
        for v in vehicles_list:
            if v.attributes['role_name'] != "hero":
                distance = math.sqrt((transform.location.x - v.get_transform().location.x) ** 2 + (
                            transform.location.y - v.get_transform().location.y) ** 2)
                if distance < 20 and (
                        int(v.get_velocity().y) or int(v.get_velocity().x) or int(v.get_velocity().z) != 0):
                    print('type:', v.type_id, 'id', v.id, 'v:', v.get_velocity())

finally:
    if set_synchronous:
        print('set mode to asynchronous')
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        tm.set_synchronous_mode(False)

    print('\ndestroying %d vehicles' % len(vehicles_id_list))
    client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_id_list])
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
    time.sleep(0.5)
