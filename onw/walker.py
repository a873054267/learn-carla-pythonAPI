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
        SetAutopilot = command.SetAutopilot
        FutureActor = command.FutureActor

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

        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walkers_list = []
        all_id = []
        #因为添加的人可能会失败，所以速度的数组需要根据成功的数量，重新设置；
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2

        batch = []
        #获取ai walker bp
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put together the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):

            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        for i in range(0, len(all_id), 2):
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

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])


        time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')

