from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import os

import os
import pickle
import dataclasses
import pathlib
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
import numpy as np

def plot(reward):
    init = reward[0]
    max_reward = [max(i) for i in reward]
    avg_reward = [np.mean(i) for i in reward]

    print(reward)
    input()
    reward = reward[1:]
    # reward = np.random.random((30,10))
    plt.plot(init,'o',color = '#C0C0C0')
    plt.plot(reward,'o',color = '#C0C0C0')
    plt.plot(max_reward,color = 'red',label = 'Best reward')
    plt.plot(max_reward,'o',color = 'red')
    plt.plot(avg_reward,color = 'gray',label = 'Avg reward')
        # plt.plot(avg_reward,'o',color = '#C0C0C0')
    # plt.ylim((0,1.2))
    plt.ylabel('Reward')
    plt.xlabel('Generation')
    plt.legend()
    plt.savefig("best_results.png")
    plt.show()


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
        print(data)
        # print("Hello World")
        return data  # in format [velocity_reward, power_reward]


path = 'Experiment_2/Offspring_2024-05-19_11-44-01/log_files'
file_path = f'generation_22.pkl'

# Open the pickle file and load the data
with open(path+'/'+file_path, 'rb') as file:
    robots = pickle.load(file)

sorted_robots = sorted(robots.items(), key=lambda x: x[1].reward, reverse=True)

count = 0
for robot_name, robot_data in sorted_robots:
    if(count == 10):
        break
    print(robot_name)
    if robot_data.reward == -10000000:
        pass
    else:
        print(f"Generation: {robot_data.generation}, Reward: {robot_data.reward}, Evaluated: {robot_data.model_dir}")
        extract_velocity_and_energy_reward(os.path.join('.',robot_data.model_dir))
        print()
    count = count+1


# offspring_path = './Experiment_3/test_vis/FrogBot_AntBot_top-z-3_FrogBot_bottom-y-8'
# extract_velocity_and_energy_reward(offspring_path)