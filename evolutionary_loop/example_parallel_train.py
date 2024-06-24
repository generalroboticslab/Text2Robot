
import os
import sys
import pathlib

from train_util import get_subdirectory_configs,new_cd
from evolutionary_loop.experiments.Experiment_1 import *
# from ..legged_env.envs.parallel_train import Launcher

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from legged_env.envs.parallel_train import Launcher


with new_cd(os.path.dirname(__file__)):
    # base_directory = "../../RobotsMakingRobots/assets/URDFsForBoxi"

    run_directory="../legged_env/envs"

    # for p in pathlib.Path(os.path.abspath(base_directory)).rglob("*.urdf"):
    #     print(p)

    base_dir_path = pathlib.Path(base_directory)
    configs = [get_subdirectory_configs(x,extra_configs) for x in base_dir_path.iterdir() if x.is_dir()]

    head= HEAD_PLAY

    str_configs = [
        " ".join([f"{key}={value}".replace(' ','') for key, value in configs[1].items()]).replace('\t', '') for k in range(len(configs))
    ]

    # for i,c in enumerate(str_configs):
    #     print(f"\"{head} {c}\",")
        
    cmd_lists = [f"{head} {c}" for c in str_configs]
    
    experiments = [dict(cmd=cmd,root_dir=f"exp_{k}",exp_env_vars=None) for k,cmd in enumerate(cmd_lists)]
    launcher = Launcher(max_parallel=4,num_gpus=1,experiments_per_gpu=2,train_dir="./output/tmp", pause_between=0)
    launcher.add(experiments)
    launcher.run()