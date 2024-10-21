import carla
import time
from utils import *
from enums import *
import logging
from carla import command

import random


def main():
    asynch = False
    synchronous_master = False
    try:
        # @todo cannot import these directly.
        SpawnActor = command.SpawnActor


        settings = world.get_settings()


        #tm的默认端口是8000
        traffic_manager = client.get_trafficmanager(8000)

        if not asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        world.apply_settings(settings)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        blueprints = get_actor_blueprints(world,'vehicle.*','ALL')
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road




        #获取地图中随机的位置
        spawn_points = []
        for i in range(3):
            spawn_point = carla.Transform()#主动生成一个transform
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)

        # 2. we spawn the walker object

        batch = []
        walker_speed = []
        blueprintsWalkers = get_actor_blueprints(world, 'walker.pedestrian.*', '2')
        percentagePedestriansRunning = 10.0      # how many pedestrians will run




        batch = []
        #获取ai walker bp

        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(3):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform()))
        results = client.apply_batch_sync(batch, True)
        all_id=[]
        # 4. we put together the walkers and controllers id to get the objects from their id
        for i in range(len(results)):

            all_id.append(results[i].actor_id)

        all_actors = world.get_actors(all_id)

        for i in range(0, len(all_id), 1):
            #步长为2，所以，走动的其实是ai控制的worker，其他的并没有移动
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))

        #车辆速度

        # all_vehicle_actors = world.get_actors(vehicles_list)
        # for actor in all_vehicle_actors:
        #     traffic_manager.update_vehicle_lights(actor, True)
        #


        traffic_manager.global_percentage_speed_difference(30.0)

        while True:
            if not asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()
    finally:

        #要将所有生成的目标对象销毁
        if not asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(all_id))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])


        time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')

