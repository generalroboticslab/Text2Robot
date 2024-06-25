import random
import os
import shutil
from util import *

def create_and_copy_offspring_folder(source_dirs, experiment_name, offspring_path):
    # Create the new directory
    os.makedirs(offspring_path, exist_ok=True)
    
    # Copy the contents of the source directories
    for source_dir in source_dirs:
        folder_name = os.path.basename(source_dir)
        destination_path = os.path.join(offspring_path, folder_name)
        
        # Create the destination directory if it doesn't exist
        os.makedirs(destination_path, exist_ok=True)

        # Iterate over files in the directory
        for root, dirs, files in os.walk(source_dir):
            for file in files:
                # Check if the file ends with .urdf or .yaml
                if file.endswith('.urdf') or file.endswith('.yaml'):
                    # Prepare the full path of the source and destination
                    source_file_path = os.path.join(root, file)
                    destination_file_path = os.path.join(destination_path, file)
                    
                    # Copy the file
                    shutil.copy2(source_file_path, destination_file_path)
                if file.endswith('.urdf'):
                    modify_urdf_file(destination_file_path, folder_name)

    # print(f"All folders have been copied to {offspring_path}")

def init(experiment_name, robot_names, bank_size, offspring_path):

    init_population = []
    axis_choices = ['x','y','z']

    while len(init_population) < 150: # Change from 150 to alter the size of the initial population, will infinitely loop if less than 150 robots in URDF_Bank

        scale = random.randint(0, bank_size/3 - 1)

        robot_name = random.choice(robot_names)
        axis = random.choice(axis_choices)

        robot_string = assemble_robot_name(top_axis=axis, bottom_axis=axis, top_scale=scale, bottom_scale=scale, top_name=robot_name, bottom_name=robot_name, baselink_name=robot_name)

        item_to_append = experiment_name + "/URDF_Bank/" + robot_string

        if item_to_append not in init_population:
            init_population.append(item_to_append)

        
    create_and_copy_offspring_folder(init_population, experiment_name, offspring_path)