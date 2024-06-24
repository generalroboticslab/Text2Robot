from init_population import init
import os, sys
import numpy as np
from util import *
from swapping import *
from gen_config_file import g_config_file
import dataclasses
from train_util import get_subdirectory_configs, normalized_velocity_command
from greatest_reward import find_greatest_reward
from extract_rewards_from_tensorboard_file import *
import pathlib
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from legged_env.envs.parallel_train import Launcher
import logging
import datetime
import pickle
import argparse
import yourdfpy
from evolutionary_loop.experiments.Experiment_4 import *

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

offspring_folder_path = os.path.join(experiment_name, "Offspring_" + timestamp)

# Create a logs directory if it doesn't exist
script_dir = os.path.dirname(os.path.abspath(__file__))

# Define the log directory relative to the script location
log_directory = os.path.join(offspring_folder_path, 'log_files')

os.makedirs(log_directory, exist_ok=True)

# Timestamp for the log file name
log_filename = f"{log_directory}/log_{timestamp}.txt"

gen = 0

# Set up logging
logging.basicConfig(filename=log_filename, level=logging.INFO, filemode='a', format='%(message)s')
logging.info("Initialized log file at: " + str(timestamp))
 
@dataclasses.dataclass
class Offspring():
    model_dir: str = None
    cmd: str = None
    run_dir: str  = None
    reward: float = None
    evaluated: bool = False
    generation: int = None
    checkpoint_file: str = None
    parents: list = dataclasses.field(default_factory=list)
    velocity_contribution: float = None
    power_contribution: float = None

robots = dict()
archive = dict()

launcher = Launcher(max_parallel=24,num_gpus=6,experiments_per_gpu=4,train_dir="./output/", pause_between=0)

bank_path = os.path.join(experiment_name,'URDF_Bank')

for model_dir in pathlib.Path(bank_path).iterdir():
    if model_dir.is_dir() and model_dir.name != 'log_files':
        robot_name= os.path.basename(model_dir)
        robot = Offspring(model_dir=model_dir)
        robot.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
        robot.cmd=get_subdirectory_configs(model_dir,extra_configs, False)
        robot.cmd["hydra.run.dir"]= robot.run_dir
        robot.generation = 0
        robots[robot_name] = robot

experiments = []

for robot in robots.values():
    if robot.evaluated is False:
        _, root_dir = os.path.split(robot.run_dir)

        # Get command line overrides to normalize task difficulty by body length
        urdf_filename = 'robot.urdf'
        path_to_urdf = os.path.join(robot.model_dir, urdf_filename)
        urdf = yourdfpy.URDF.load(path_to_urdf)
        velocity_commands = " " + normalized_velocity_command(urdf)

        experiments.append(
            dict(
                cmd= HEAD_TRAIN + " ".join([f"{key}={value}".replace(' ','') for key, value in robot.cmd.items()]).replace('\t', '') + velocity_commands,
                root_dir = root_dir,
                exp_env_vars=None
            )
        )

launcher.add(experiments)
launcher.run() 

for robot in robots.values():
    if robot.evaluated is False:
        path_to_nn = find_nn_path(robot.run_dir)
        reward, checkpoint_file = find_greatest_reward(path_to_nn)

        # Collect and record the individual reward contributions
        reward_contributions = extract_velocity_and_energy_reward(robot.model_dir)
        robot.velocity_contribution = reward_contributions[0]
        
        robot.power_contribution = reward_contributions[1]

        log_print(reward)

        # print(reward)
        robot.reward = reward
        robot.checkpoint_file = checkpoint_file
        robot.evaluated = True

        file_path = f"{robot.model_dir}/reward.txt"
    
        with open(file_path, "w") as f:
            f.write(f"{reward:.2f}")

    # Send all info to a current generation pickle file
    full_dict_of_all_bots = {**robots, **archive}
    pickle_directory = log_directory
    filename = f"generation_{gen}.pkl"
    full_path = os.path.join(log_directory, filename)

    # Pickling the dictionary
    with open(full_path, 'wb') as file:
        pickle.dump(full_dict_of_all_bots, file)

    log_print(f"Sent all bots to {filename}")