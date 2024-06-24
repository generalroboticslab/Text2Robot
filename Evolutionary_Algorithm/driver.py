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

max_generations = 55 # Customize how many evolutionary generations you want to run

inform_based_on_energy = False # You can set one of these options to true to evolve for a particular preference
inform_based_on_velocity = False

rough_terrain = False # Set to true to test robots on rough terrain (longer training time)

from Evolutionary_Algorithm.experiments.Example_Experiment import * # Change this to the experiment preferences file corresponding to your experiment

gen = 0

bank_size = 30 # 30 robots generated per mesh (You can include as many meshes (30 * prompts = # of models), but it should be divisible by 30)


timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
offspring_folder_path = os.path.join(experiment_name, "Offspring_" + timestamp) # Experiment results are stored here

script_dir = os.path.dirname(os.path.abspath(__file__))

# Create a logging file
log_directory = os.path.join(offspring_folder_path, 'log_files')
os.makedirs(log_directory, exist_ok=True)
log_filename = f"{log_directory}/log_{timestamp}.txt"
# Set up logging
logging.basicConfig(filename=log_filename, level=logging.INFO, filemode='a', format='%(message)s')
logging.info("Initialized log file at: " + str(timestamp))

init(experiment_name, robot_names, bank_size, offspring_folder_path) # Init 200 robots from the URDF_Bank to be the first generation

# Robots are pickled according to this file, and any of the information can be retroactively retrieved.
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

# Increase experiments_per_gpu and max_parallel if you have more than one GPU for training
launcher = Launcher(max_parallel=4,num_gpus=1,experiments_per_gpu=4,train_dir="./output/", pause_between=0)

for model_dir in pathlib.Path(offspring_folder_path).iterdir():
    if model_dir.is_dir() and model_dir.name != 'log_files':
        robot_name= os.path.basename(model_dir)
        robot = Offspring(model_dir=model_dir)
        robot.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
        robot.cmd=get_subdirectory_configs(model_dir,extra_configs, False, rough_terrain)
        robot.cmd["hydra.run.dir"]= robot.run_dir
        robot.generation = gen
        robot.evaluated = False
        robots[robot_name] = robot


while(gen < max_generations + 1): # Continue evolving until termination criteria is met (max_generations is reached)
    experiments = []
    for robot in robots.values():
        if robot.evaluated is False:
            _, root_dir = os.path.split(robot.run_dir)

            # Get command line overrides to normalize task difficulty by body length
            urdf_filename = os.path.basename(robot.model_dir) + '.urdf'
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
            
            robot.velocity_contribution = 20 * reward_contributions[0]
            robot.power_contribution = -10 * reward_contributions[1]

            if inform_based_on_energy and (not reward == -10000000): # If evolving with energy or velocity preference, adjust total reward
                reward = reward + robot.power_contribution
            if inform_based_on_velocity and (not reward == -10000000):
                reward = reward + robot.velocity_contribution

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

    sorted_robots = sorted(robots.items(), key=lambda x: x[1].reward, reverse=True)

    best_bot = sorted_robots[0][0] 

    top_robots = dict(sorted_robots[:100])
    archive.update(sorted_robots[100:])
    robots = dict(top_robots)
    archive = dict(archive)
    
    # Check to see if termination criteria is met:
    if gen >= max_generations:
        print(f'Evolution complete. Selected bot: {best_bot}')
        break
    else:
        gen = gen + 1

    count = 0 # Use to make sure genetic isolation doesn't occur
    while len(robots) < 100:  # Use crossover to create 4 new offspring using combinations of the best previous generation
        while True:
            parent_1 = random.choice(list(robots.keys()))
            parent_2 = random.choice(list(robots.keys()))
            
            if parent_1 != parent_2: # Make sure you aren't swapping a robot with itself
                break
        
        swap = swap_legs_from_urdf(parent_1, parent_2, offspring_folder_path, False)

        swap_link_or_joint_choice = random.choice(['link','joint'])
        if swap_link_or_joint_choice == 'link':
            generated_bot_1 = swap.swap_links(dest_path=offspring_folder_path)
        if swap_link_or_joint_choice == 'joint':
            generated_bot_1 = swap.swap_joints(dest_path=offspring_folder_path)
        
        # Add new offspring to robots
        if not generated_bot_1 == None:
            robot_1_name= generated_bot_1
            model_dir = os.path.join(offspring_folder_path, robot_1_name)
            robot_1 = Offspring(model_dir=model_dir)
            robot_1.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
            robot_1.cmd=get_subdirectory_configs(pathlib.Path(model_dir),extra_configs)
            robot_1.cmd["hydra.run.dir"]= robot_1.run_dir
            robot_1.generation = gen
            robot_1.parents.append(robots[parent_1])
            robot_1.parents.append(robots[parent_2])
            robots[robot_1_name] = robot_1
        
        count = count + 1 # Increment infinite loop checker

        if count > 100: # Run a full mutation step
            for robot_1_name in list(robots.keys()):
                # Mutate link with a 15% chance
                random_range = random.random()
                random_level = random.randint(1,2) # Choose which level to mutate (if a mutation occurs)

                if random_range < 0.15:
                    robot_2_name = create_mutate_link_robot_string(robot_1_name, bank_size, random_level)
                    swap = swap_legs_from_urdf(robot_1_name, robot_2_name, offspring_folder_path, True)
                    generated_robot = swap.mutate_links(dest_path=offspring_folder_path, level=random_level)

                    if not generated_robot == None:
                        # moved_bot = robots.pop(robot_1_name)
                        # archive[robot_1_name] = moved_bot

                        robot_name = generated_robot
                        model_dir = os.path.join(offspring_folder_path, robot_name)
                        robot = Offspring(model_dir=model_dir)
                        robot.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
                        robot.cmd=get_subdirectory_configs(pathlib.Path(model_dir),extra_configs)
                        robot.cmd["hydra.run.dir"]= robot.run_dir
                        robot.generation = gen
                        robot.parents.append(robots[robot_1_name])
                        robots[robot_name] = robot

                # Mutate joint with a 15% chance
                elif random_range < 0.55:
                    robot_2_name = create_mutate_joint_robot_string(robot_1_name, bank_size, random_level)
                    swap = swap_legs_from_urdf(robot_1_name, robot_2_name, offspring_folder_path, True)
                    generated_robot = swap.mutate_joints(dest_path=offspring_folder_path, level=random_level)

                    if not generated_robot == None:
                        # moved_bot = robots.pop(robot_1_name)
                        # archive[robot_1_name] = moved_bot

                        robot_name = generated_robot
                        model_dir = os.path.join(offspring_folder_path, robot_name)
                        robot = Offspring(model_dir=model_dir)
                        robot.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
                        robot.cmd=get_subdirectory_configs(pathlib.Path(model_dir),extra_configs)
                        robot.cmd["hydra.run.dir"]= robot.run_dir
                        robot.generation = gen
                        robot.parents.append(robots[robot_1_name])
                        robots[robot_name] = robot

                elif random_range < 0.7:
                    # log_print("SWAPPING ROBOT NAMEs")
                    robot_2_name = create_mutate_name_robot_string(robot_1_name, bank_size, random_level, robot_names)
                    # log_print("Old bot name: " + robot_1_name + "   New bot name: " + robot_2_name)
                    if not (robot_2_name == None):
                        swap = swap_legs_from_urdf(robot_1_name, robot_2_name, offspring_folder_path, True)
                        generated_robot = swap.mutate_links(dest_path=offspring_folder_path, level=random_level)

                        if not generated_robot == None:
                            # moved_bot = robots.pop(robot_1_name)
                            # archive[robot_1_name] = moved_bot

                            robot_name = generated_robot
                            model_dir = os.path.join(offspring_folder_path, robot_name)
                            robot = Offspring(model_dir=model_dir)
                            robot.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
                            robot.cmd=get_subdirectory_configs(pathlib.Path(model_dir),extra_configs)
                            robot.cmd["hydra.run.dir"]= robot.run_dir
                            robot.generation = gen
                            robot.parents.append(robots[robot_1_name])
                            robots[robot_name] = robot
                elif random_range < 0.95:
                    # Swapping baselinks
                    robot_2_name = create_mutate_baselink_robot_string(robot_1_name, bank_size, robot_names)

                    if not (robot_2_name == None):
                        swap = swap_legs_from_urdf(robot_1_name, robot_2_name, offspring_folder_path, True)
                        generated_robot = swap.mutate_baselink(dest_path=offspring_folder_path)

                        if not generated_robot == None:
                            # moved_bot = robots.pop(robot_1_name)
                            # archive[robot_1_name] = moved_bot

                            robot_name = generated_robot
                            model_dir = os.path.join(offspring_folder_path, robot_name)
                            robot = Offspring(model_dir=model_dir)
                            robot.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
                            robot.cmd=get_subdirectory_configs(pathlib.Path(model_dir),extra_configs)
                            robot.cmd["hydra.run.dir"]= robot.run_dir
                            robot.generation = gen
                            robot.parents.append(robots[robot_1_name])
                            robots[robot_name] = robot

            count = 0 # Reset infinite loop checker

    # Random Chance for Mutations

    stop_generating_flag = False

    while True:
        for robot_1_name in list(robots.keys()):
            if len(robots) > 199:
                stop_generating_flag = True
                break
            # Mutate link with a 15% chance
            random_range = random.random()
            random_level = random.randint(1,2) # Choose which level to mutate (if a mutation occurs)

            if random_range < 0.15:
                robot_2_name = create_mutate_link_robot_string(robot_1_name, bank_size, random_level)
                swap = swap_legs_from_urdf(robot_1_name, robot_2_name, offspring_folder_path, True)
                generated_robot = swap.mutate_links(dest_path=offspring_folder_path, level=random_level)

                if not generated_robot == None:
                    # moved_bot = robots.pop(robot_1_name)
                    # archive[robot_1_name] = moved_bot

                    robot_name = generated_robot
                    model_dir = os.path.join(offspring_folder_path, robot_name)
                    robot = Offspring(model_dir=model_dir)
                    robot.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
                    robot.cmd=get_subdirectory_configs(pathlib.Path(model_dir),extra_configs)
                    robot.cmd["hydra.run.dir"]= robot.run_dir
                    robot.generation = gen
                    robot.parents.append(robots[robot_1_name])
                    robots[robot_name] = robot

            # Mutate joint with a 15% chance
            elif random_range < 0.45:
                robot_2_name = create_mutate_joint_robot_string(robot_1_name, bank_size, random_level)
                swap = swap_legs_from_urdf(robot_1_name, robot_2_name, offspring_folder_path, True)
                generated_robot = swap.mutate_joints(dest_path=offspring_folder_path, level=random_level)

                if not generated_robot == None:
                    # moved_bot = robots.pop(robot_1_name)
                    # archive[robot_1_name] = moved_bot

                    robot_name = generated_robot
                    model_dir = os.path.join(offspring_folder_path, robot_name)
                    robot = Offspring(model_dir=model_dir)
                    robot.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
                    robot.cmd=get_subdirectory_configs(pathlib.Path(model_dir),extra_configs)
                    robot.cmd["hydra.run.dir"]= robot.run_dir
                    robot.generation = gen
                    robot.parents.append(robots[robot_1_name])
                    robots[robot_name] = robot

            elif random_range < 0.65:
                # log_print("SWAPPING ROBOT NAMEs")
                robot_2_name = create_mutate_name_robot_string(robot_1_name, bank_size, random_level, robot_names)
                # log_print("Old bot name: " + robot_1_name + "   New bot name: " + robot_2_name)
                if not (robot_2_name == None):
                    swap = swap_legs_from_urdf(robot_1_name, robot_2_name, offspring_folder_path, True)
                    generated_robot = swap.mutate_links(dest_path=offspring_folder_path, level=random_level)

                    if not generated_robot == None:
                        # moved_bot = robots.pop(robot_1_name)
                        # archive[robot_1_name] = moved_bot

                        robot_name = generated_robot
                        model_dir = os.path.join(offspring_folder_path, robot_name)
                        robot = Offspring(model_dir=model_dir)
                        robot.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
                        robot.cmd=get_subdirectory_configs(pathlib.Path(model_dir),extra_configs)
                        robot.cmd["hydra.run.dir"]= robot.run_dir
                        robot.generation = gen
                        robot.parents.append(robots[robot_1_name])
                        robots[robot_name] = robot

            elif random_range < 0.90:
                    # Swapping baselinks
                    robot_2_name = create_mutate_baselink_robot_string(robot_1_name, bank_size, robot_names)

                    if not (robot_2_name == None):
                        swap = swap_legs_from_urdf(robot_1_name, robot_2_name, offspring_folder_path, True)
                        generated_robot = swap.mutate_baselink(dest_path=offspring_folder_path)

                        if not generated_robot == None:
                            # moved_bot = robots.pop(robot_1_name)
                            # archive[robot_1_name] = moved_bot

                            robot_name = generated_robot
                            model_dir = os.path.join(offspring_folder_path, robot_name)
                            robot = Offspring(model_dir=model_dir)
                            robot.run_dir= os.path.abspath(os.path.join(model_dir,'run'))
                            robot.cmd=get_subdirectory_configs(pathlib.Path(model_dir),extra_configs)
                            robot.cmd["hydra.run.dir"]= robot.run_dir
                            robot.generation = gen
                            robot.parents.append(robots[robot_1_name])
                            robots[robot_name] = robot

        if stop_generating_flag == True: # Break out of mutation loop once 8 new robots have been generated
            break

    g_config_file(offspring_folder_path) # Update YAML Files of all robots in the offspring folder

    log_information(gen, robots, archive)
