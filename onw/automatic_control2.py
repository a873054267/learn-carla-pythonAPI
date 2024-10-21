import glob
import os
import sys

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    carlAPI = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '\\carla'
    print(carlAPI)
    sys.path.append(carlAPI)
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

from agents.navigation.behavior_agent import BehaviorAgent

import random

def main():
    try:
        client = carla.Client('localhost', 3000)
        client.set_timeout(3.0)

        # Retrieve the world that is currently running
        world = client.get_world()

        origin_settings = world.get_settings()

        # set sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # read all valid spawn points
        all_default_spawn = world.get_map().get_spawn_points()
        # randomly choose one as the start point
        spawn_point = random.choice(all_default_spawn) if all_default_spawn else carla.Transform()

        # create the blueprint library
        ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
        ego_vehicle_bp.set_attribute('role_name', "hero")
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # spawn the vehicle
        vehicle = world.spawn_actor(ego_vehicle_bp, spawn_point)

        # we need to tick the world once to let the client update the spawn position
        world.tick()

        # create the behavior agent
        agent = BehaviorAgent(vehicle, behavior='normal')

        # set the destination spot
        spawn_points = world.get_map().get_spawn_points()
        random.shuffle(spawn_points)

        # to avoid the destination and start position same
        if spawn_points[0].location != agent.vehicle.get_location():
            destination = spawn_points[0]
        else:
            destination = spawn_points[1]

        # generate the route
        agent.set_destination(agent.vehicle.get_location(), destination.location, clean=True)

        while True:
            agent.update_information()

            world.tick()
            # top view
            spectator = world.get_spectator()
            transform = vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=40),
                                                    carla.Rotation(pitch=-90)))

            speed_limit = vehicle.get_speed_limit()
            agent.get_local_planner().set_speed(speed_limit)

            control = agent.run_step(debug=True)
            vehicle.apply_control(control)
            if len(agent.get_local_planner().waypoints_queue) == 0:
                print("Target reached, mission accomplished...")
                break

    finally:
        world.apply_settings(origin_settings)
        vehicle.destroy()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
