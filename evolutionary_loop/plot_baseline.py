import os
import pickle
import dataclasses
import pathlib
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
import numpy as np

path = 'Experiment_4/Offspring_2024-05-23_11-18-11/log_files'

def plot(reward):
    # Ensure reward is a 1D array
    reward = np.array(reward)
    
    # Generate x values as indices of the reward array
    x_values = range(0, len(reward))
    
    # Plot the values
    plt.plot(x_values, reward, 'o', color='blue', label='Reward')
    
    # Add labels and legend
    plt.title('Our Rewards after Initialization (No Evolutions)')
    plt.ylabel('Reward')
    plt.ylim((-5,110))
    plt.xlabel('Robots')
    
    # Save and show the plot
    plt.savefig("reward_plot.png")
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

file_path = f'generation_0.pkl'
print(file_path)
# Open the pickle file and load the data
with open(path+'/'+file_path, 'rb') as file:
    my_dictionary = pickle.load(file)

# Assuming my_dictionary is a dictionary of Offspring objects
print("Loaded dictionary contains:")
rewards = []
for key, offspring in my_dictionary.items():
    print(f"Generation: {offspring.generation}, Reward: {offspring.reward}, Evaluated: {offspring.evaluated}")
    
    if offspring.reward == -10000000:
        # reward_sub.append(0)
        pass
    else:
        rewards.append(offspring.reward)

plot(rewards)