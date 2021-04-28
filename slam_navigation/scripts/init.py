#!/usr/bin/env python
from pyrobot import Robot
import numpy as np
base_config_dict={'base_controller': 'movebase'}

robot = Robot('locobot', base_config=base_config_dict)

robot.camera.reset()
# robot.arm.go_home()
# tilt = -np.pi/2
# robot.camera.set_tilt(tilt, wait=True)
# target_position = [0.5, 0.0, 0] # rotate on-spot by 90 degrees

# robot.base.go_to_relative(target_position, smooth=False, close_loop=True)

# target_position = [0, 0.0, 1.5707] # rotate on-spot by 90 degrees

# robot.base.go_to_relative(target_position, smooth=False, close_loop=True)

# target_position = [0, 0.0, 1.5707] # rotate on-spot by 90 degrees

# robot.base.go_to_relative(target_position, smooth=False, close_loop=True)

# target_position = [0, 0.0, 1.5707] # rotate on-spot by 90 degrees

# robot.base.go_to_relative(target_position, smooth=False, close_loop=True)

# target_position = [0, 0.0, -1.5707] # rotate on-spot by 90 degrees

# robot.base.go_to_relative(target_position, smooth=False, close_loop=True)


# target_position = [1, 0.0, 0] # rotate on-spot by 90 degrees

# robot.base.go_to_relative(target_position, smooth=False, close_loop=True)