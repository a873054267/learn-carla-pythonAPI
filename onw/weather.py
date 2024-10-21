import carla
from utils import *


#cloudiness=10.0,precipitation=10.0, fog_density=10.0)
weather = carla.WeatherParameters(0,100,1)
weather.sun_azimuth_angle=10
#控制阳光照射角度
weather.sun_altitude_angle=176
world.set_weather(weather)

