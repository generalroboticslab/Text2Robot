from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import os
import pickle
import dataclasses
import pathlib
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
import numpy as np

# Select which experiment, and which generation to run
path = 'Experiment_3/Offspring_2024-05-19_11-26-58/log_files'
file_path = f'generation_69.pkl'


@dataclasses.dataclass
class Offspring:
    model_dir: str = None
    cmd: str = None
    run_dir: str = None
    reward: float = None
    evaluated: bool = False
    generation: int = None
    checkpoint_file: str = None
    parents: list = dataclasses.field(default_factory=list)
reward = []

def get_tb_data(tb_log_dir, fields):
    """
    This function will get the data from the tensorboard logs and return a list with the last value of each field

    Args:
    tb_log_dir: base directory where the tensorboard logs are stored
    fields: list of fields to extract from the logs
    """
    data_array = []
    
    log_directory = tb_log_dir

    print('Loading data ...')
    event_acc = EventAccumulator(log_directory)
    event_acc.Reload()

    for field in fields:
        scalar = event_acc.Scalars(field)
        # get the last value and store it in the array
        data_array.append(scalar[-1].value)

    return data_array

def extract_velocity_and_energy_reward(offspring_path): # Intended to be called on robot.model_dir

    fields = ['Episode/rew_lin_vel_xy_raw', 'Episode/rew_joint_pow_raw']

    path_to_runs_folder = os.path.join(offspring_path,'run', 'runs')

    if os.path.exists(path_to_runs_folder) and os.path.isdir(path_to_runs_folder):

        specific_dir = os.listdir(path_to_runs_folder).pop()

        full_path_to_summaries_folder = os.path.join(path_to_runs_folder, specific_dir, 'summaries')

        print(full_path_to_summaries_folder)

        data = get_tb_data(full_path_to_summaries_folder, fields)
        data[0] = np.sqrt(-0.25 * np.log(data[0]))
        # print(data)
        # print("Hello World")
        return data  # in format [velocity_reward, power_reward]

# Open the pickle file and load the data
with open(path+'/'+file_path, 'rb') as file:
    robots = pickle.load(file)

sorted_robots = sorted(robots.items(), key=lambda x: x[1].reward, reverse=True)

rewards = []
velocity_rewards = []
energy_rewards = []
generations = []
models = []
bots = []

count = 0
for robot_name, robot_data in sorted_robots:
    if count == 100:
        break
    if robot_data.reward != -10000000:
        # Store the generation number and rewards
        generations.append(robot_data.generation)
        rewards.append(robot_data.reward)
        vel_reward, ene_reward = extract_velocity_and_energy_reward(os.path.join('.', robot_data.model_dir))
        velocity_rewards.append(vel_reward)
        energy_rewards.append(ene_reward)
        models.append(robot_data.model_dir)
        bots.append(robot_data)
    count += 1

# You might need to convert lists to numpy arrays for easier handling
rewards = np.array(rewards)
velocity_rewards = np.array(velocity_rewards)
energy_rewards = np.array(energy_rewards)

# Save to a pickle file
reward_path = os.path.join(path, 'reward_data.pkl')

with open(reward_path, 'wb') as f:
    pickle.dump({
        'rewards': rewards,
        'velocity_rewards': velocity_rewards,
        'energy_rewards': energy_rewards,
        'models': models,
        'robots': bots
    }, f)