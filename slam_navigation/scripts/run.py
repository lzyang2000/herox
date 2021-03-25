#!/usr/bin/env python
from pyrobot import Robot

base_config_dict={'base_controller': 'movebase'}

robot = Robot('locobot', base_config=base_config_dict)

# robot.camera.reset()
# robot.arm.go_home()

# target_position = [0, 0.0, 1.5707] # rotate on-spot by 90 degrees

# robot.base.go_to_relative(target_position, smooth=False, close_loop=True)

target_position = [1.8, 0, 0] # rotate on-spot by 90 degrees

robot.base.go_to_relative(target_position, smooth=False, close_loop=True)

# target_position = [0, 0.0, 1.5707] # rotate on-spot by 90 degrees

# robot.base.go_to_relative(target_position, smooth=False, close_loop=True)

# target_position = [0, 0.0, 1.5707] # rotate on-spot by 90 degrees

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