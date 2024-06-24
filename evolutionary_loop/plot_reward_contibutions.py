from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import os
import pickle
import dataclasses
import pathlib
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
import numpy as np

# Select which experiment, and which generation to run
pickle_path = 'Experiment_3/Offspring_2024-05-19_11-26-58/log_files/reward_data.pkl'

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

def plot_rewards_and_contributions(p_frontX, p_frontY, velocity_rewards, energy_rewards, title):
    plt.figure(figsize=(10, 6))
    plt.scatter(velocity_rewards, energy_rewards, color='blue', label='Data Points')
    plt.plot(p_frontX, p_frontY, color='red', marker='o', label='Pareto Front')
    plt.title(title + 'Energy Contribution vs Velocity Contribution')
    plt.xlabel('Velocity Contribution')
    plt.ylabel('Energy Contribution')
    plt.legend()
    plt.gca().invert_yaxis()  # Invert y-axis to show lower energy as better
    plt.savefig("reward_breakdown.png")
    plt.grid(True)

def pareto_frontier(Xs, Ys, maxX = True, maxY = True):
    """Sort the list in either ascending or descending based on Ys.
       maxX: True if high X values are better, otherwise False.
       maxY: True if high Y values are better, otherwise False.
    """
    # Sort the list in ascending order of X and descending order of Y
    myList = sorted([[Xs[i], Ys[i]] for i in range(len(Xs))], reverse=maxX)
    # Start the Pareto frontier with the first value in the sorted list
    p_front = [myList[0]]
    # Loop through the sorted list
    for pair in myList[1:]:
        if maxY:
            if pair[1] >= p_front[-1][1]: # Look for higher Y value
                p_front.append(pair)
        else:
            if pair[1] <= p_front[-1][1]: # Look for lower Y value
                p_front.append(pair)
    # Turn resulting pairs back into a list of Xs and Ys
    p_frontX = [pair[0] for pair in p_front]
    p_frontY = [pair[1] for pair in p_front]
    return p_frontX, p_frontY

with open(pickle_path, 'rb') as f:
    data = pickle.load(f)

# Access your data
rewards = data['rewards']
velocity_rewards = data['velocity_rewards']
energy_rewards = data['energy_rewards']
models = data['models']

title = "Top 100 Bots (at generation 45): "

p_frontX, p_frontY = pareto_frontier(velocity_rewards, energy_rewards, maxX=True, maxY=False)

plot_rewards_and_contributions(p_frontX, p_frontY, velocity_rewards, energy_rewards, title)

max_velocity_index = np.argmax(velocity_rewards)

max_velocity_value = velocity_rewards[max_velocity_index]

max_velocity_model = models[max_velocity_index]

print("Bot with max velocity_tracking: ", max_velocity_model)

min_energy_index = np.argmin(energy_rewards)

min_energy_value = energy_rewards[min_energy_index]

min_energy_model = models[min_energy_index]

print("Bot with min energy: ", min_energy_model)

    