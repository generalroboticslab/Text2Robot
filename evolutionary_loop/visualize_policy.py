import os, sys
import dataclasses
from train_util import get_subdirectory_configs
from experiments.Experiment_2 import *
import pathlib
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from legged_env.envs.parallel_train import Launcher

offspring_folder_path = "/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/vis"
# checkpoint_paths_array = ["/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/vis/Best0/run/runs/RobotDog_23-09-47-04/nn/RobotDog.pth",
#                     "/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/vis/Best2/run/runs/RobotDog_23-11-37-49/nn/RobotDog.pth",
#                     "/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/vis/Best3/run/runs/RobotDog_23-12-36-33/nn/RobotDog.pth",
#                     "/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/vis/Best3_1/run/runs/RobotDog_23-15-52-12/nn/RobotDog.pth",
#                     "/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/vis/Best3_2/run/runs/RobotDog_23-23-07-24/nn/RobotDog.pth",
#                     "/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/vis/Best6/run/runs/RobotDog_24-08-58-10/nn/RobotDog.pth",
#                     "/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/vis/Best7/run/runs/RobotDog_24-15-45-16/nn/RobotDog.pth"]
checkpoint_paths_array = ['/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/assets/ShoeBot_BugBot3_top-z-5_BugBot1_bottom-x-6/run/runs/RobotDog_16-22-50-22/nn/RobotDog.pth']
# model_dir_array = [pathlib.Path('vis/Best0'),
#              pathlib.Path('vis/Best2'),
#              pathlib.Path('vis/Best3'),
#              pathlib.Path('vis/Best3_1'),
#              pathlib.Path('vis/Best3_2'),
#              pathlib.Path('vis/Best6'),
#              pathlib.Path('vis/Best7')]
model_dir_array = [pathlib.Path('/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/assets/ShoeBot_BugBot3_top-z-5_BugBot1_bottom-x-6')]

num_vis = len(model_dir_array)
             

@dataclasses.dataclass
class Offspring():
    model_dir: str = None
    cmd: str = None
    run_dir: str  = None
    reward: float = None
    evaluated: bool = False
    generation: int = None
    checkpoint_file: str = None

robots = dict()
archive = dict()

launcher = Launcher(max_parallel=4,num_gpus=1,experiments_per_gpu=2,train_dir="./output/", pause_between=0)


# for model_dir in pathlib.Path(offspring_folder_path).iterdir():

for i in range(num_vis + 1):
    robots = dict()

    model_dir = model_dir_array[i]
    checkpoint_path = checkpoint_paths_array[i]

    if model_dir.is_dir() and not model_dir.name=='log_files':
        robot_name= os.path.basename(model_dir)
        robot = Offspring(model_dir=model_dir)
        robot.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
        robot.cmd=get_subdirectory_configs(model_dir,extra_configs, False, True)
        robot.cmd["hydra.run.dir"]= robot.run_dir
        robot.generation = 0
        robots[robot_name] = robot



    experiments = []
    for robot in robots.values():
        if robot.evaluated is False:
            _, root_dir = os.path.split(robot.run_dir)
            experiments.append(
                dict(
                    cmd= HEAD_PLAY + "checkpoint=" + checkpoint_path + " " + " ".join([f"{key}={value}".replace(' ','') for key, value in robot.cmd.items()]).replace('\t', ''),
                    root_dir = root_dir,
                    exp_env_vars=None
                )
            )
            # print(HEAD_PLAY + "checkpoint=" + checkpoint_path + " " + " ".join([f"{key}={value}".replace(' ','') for key, value in robot.cmd.items()]).replace('\t', ''))
            

    launcher.add(experiments)
    launcher.run() 