import os
import sys
import yourdfpy

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from legged_env.assets.urdf_util import write_urdf

bot_name = 'ShoeBot_BugBot3_top-z-5_BugBot1_bottom-x-6'

old_urdf_path = f'/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/assets/{bot_name}/{bot_name}.urdf'
new_urdf_path = f'/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/assets/new_{bot_name}/{bot_name}.urdf'
urdf = yourdfpy.URDF.load(old_urdf_path)  

write_urdf(urdf, new_urdf_path, old_urdf_path, True)