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
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)
        batch = []
        vehicles_list = []
        hero=False
        actorVehicle = None
        number_of_vehicles=30
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
            if n == 0:
                # 如果是hero代表为自我控制车辆，在大型地图和超大交通时，可以变更为局部加载和局部碰撞计算
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')
                # spawn the cars and set their autopilot and light state all together
            item = SpawnActor(blueprint, transform)
            item.then(SetAutopilot(FutureActor, True, traffic_manager.get_port()))

            batch.append(item)

        #由于批量添加物体到场景中，异步添加可能会报错
        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        #车辆速度

        # all_vehicle_actors = world.get_actors(vehicles_list)
        # for actor in all_vehicle_actors:
        #     traffic_manager.update_vehicle_lights(actor, True)
        #


        traffic_manager.global_percentage_speed_difference(30.0)
        spectator = world.get_spectator()

        while True:
            if not asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()
            if(actorVehicle is None):
                actorVehicle =  world.get_actors(vehicles_list)[0]
            transform = actorVehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                                    carla.Rotation(yaw=int(transform.rotation.yaw), roll=0, pitch=-90)))
    finally:

        #要将所有生成的目标对象销毁
        if not asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])



        time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')

