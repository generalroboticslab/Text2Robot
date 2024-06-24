import os
import pickle
import dataclasses
import pathlib
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
import numpy as np

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


file_path = '/home/grl/Documents/RobotsMakingRobots/figure_generation/data/Experiment_1_2/log_files/generation_20.pkl'
print(file_path)
# Open the pickle file and load the data
with open(file_path, 'rb') as file:
    my_dictionary = pickle.load(file)

# Assuming my_dictionary is a dictionary of Offspring objects
print("Loaded dictionary contains:")
reward_sub=[]
for key, offspring in my_dictionary.items():
    print(f"Generation: {offspring.generation}, Reward: {offspring.reward}, Evaluated: {offspring.evaluated}, Command: {offspring.cmd}")
    
    break