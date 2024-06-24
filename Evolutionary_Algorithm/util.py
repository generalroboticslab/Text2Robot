import random
import os
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import re
import logging

def modify_urdf_file(file_path, new_directory_name):
    # Parse the URDF file
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Define the prefix to match
    prefix_to_match = "simple_meshes/"

    # Iterate over all elements and modify 'filename' attributes
    for element in root.iter():
        if 'filename' in element.attrib:
            old_filename = element.attrib['filename']
            # Check if the filename starts with the specific prefix to be replaced
            if old_filename.startswith(prefix_to_match) and '..' not in old_filename:
                # Extract the specific part after the prefix
                specific_part = old_filename[len(prefix_to_match):]
                # Construct the new filename using the provided directory string
                new_filename = f"../../URDF_Bank/{new_directory_name}/simple_meshes/{specific_part}"
                # Update the attribute
                element.attrib['filename'] = new_filename

    # Write the modified tree back to the file
    tree.write(file_path)

def evaulated(folder_path):
    return os.path.isfile(os.path.join(folder_path, "reward.txt"))

def create_reward_file(folder_path):
    random_reward = random.uniform(1, 20)
    file_path = f"{folder_path}/reward.txt"
    
    with open(file_path, "w") as f:
        f.write(f"{random_reward:.2f}")
    
    # print(f"Created {file_path} with reward: {random_reward:.2f}")

def read_float_from_file(file_path):
    try:
        with open(file_path, "r") as f:
            value = float(f.read().strip())
        return value
    except ValueError:
        print(f"Error: The file at {file_path} does not contain a valid float.")
    except FileNotFoundError:
        print(f"Error: The file at {file_path} was not found.")
    except Exception as e:
        print(f"Error: An unexpected error occurred: {e}")
    return None

def delete_directory(directory_path):
    try:
        shutil.rmtree(directory_path)
        # print(f"Directory {directory_path} deleted successfully")
    except FileNotFoundError:
        print(f"Directory {directory_path} not found")
    except PermissionError:
        print(f"No permission to delete {directory_path}")
    except Exception as e:
        print(f"Error occurred while deleting directory {directory_path}: {e}")

def create_directory(directory_path):
    directory = Path(directory_path)
    if directory.exists():
        return False
    else:
        try:
            directory.mkdir(parents=True, exist_ok=True)
            # print(f"Directory {directory_path} created successfully")
            return True
        except OSError as e:
            print(f"Error occurred while creating directory {directory_path}: {e}")


def truncate_before_final_slash(s):
    return s.rsplit('/', 1)[-1]

def extract_robot_features(robot_string):
    # Define a pattern that specifically captures the names, axis, and scale
    # This regex ensures that no leading underscore is included in the names
    # print("IN EXTRACT FEATURES")
    parts = re.split(r'[-_]', robot_string)

    # Initialize default values
    baselink, top_scale, bottom_scale, top_axis, bottom_axis, top_name, bottom_name = None, None, None, None, None, None, None
    
    if(not len(parts) == 9):
        print("Error: Unexpected robot name: ", robot_string)
        exit()

    baselink = parts[0]
    top_name = parts[1]
    top_axis = parts[3]
    top_scale = parts[4]
    bottom_name = parts[5]
    bottom_axis = parts[7]
    bottom_scale = parts[8]

    # Validate the results to ensure all data was captured properly
    if None in [baselink, top_scale, bottom_scale, top_axis, bottom_axis, top_name, bottom_name]:
        print("Error: Missing required robot features in the input string.")
    
    # print("original_string: ", robot_string)
    # print("Top_scale: ", top_scale, 
    #       "  Bottom_Scale: ", bottom_scale, 
    #       "  Top Axis: ", top_axis,
    #       "  Bottom_Axis: ", bottom_axis,
    #       "  Top_Name: ", top_name,
    #       "  Bottom_Name: ", bottom_name)
    
    return {
        'base_link_name' : baselink,
        'top_scale': top_scale,
        'bottom_scale': bottom_scale,
        'top_axis': top_axis,
        'bottom_axis': bottom_axis,
        'top_name': top_name,
        'bottom_name': bottom_name
    }

def assemble_robot_name(top_scale, bottom_scale, top_axis, bottom_axis, top_name, bottom_name, baselink_name):
    # Construct the parts of the robot name using the provided parameters
    print("Top_scale: ", top_scale, 
          "  Bottom_Scale: ", bottom_scale, 
          "  Top Axis: ", top_axis,
          "  Bottom_Axis: ", bottom_axis,
          "  Top_Name: ", top_name,
          "  Bottom_Name: ", bottom_name,
          "  Baselink_Name: ", baselink_name)
    
    top_part = f"{top_name}_top-{top_axis}-{top_scale}"
    bottom_part = f"{bottom_name}_bottom-{bottom_axis}-{bottom_scale}"
    
    # Assemble the full robot name by combining the top and bottom parts
    robot_name = f"{baselink_name}_{top_part}_{bottom_part}"
    
    print("From assemble_robot_name: " + robot_name)
    return robot_name

def change_axis(input_string):
    # Define the possible choices
    choices = ['x', 'y', 'z']
    # Remove the input string from the list of choices
    choices.remove(input_string)
    # Return a random choice from the remaining options
    return random.choice(choices)

def create_mutate_link_robot_string(robot_1, bank_size, level): # This util.ity creates a bank string with a mutated limb length. Same length for top and bot, pass requested level to mutate_links()
    r1_features = extract_robot_features(robot_1)

    upper_bound = (bank_size / 3) - 1
    target_level_name = None

    if(level == 1):
        # mutate upper leg
        current_leg_scale = r1_features['top_scale']
        target_level_name = r1_features['top_name']

    if(level == 2):
        # mutate upper leg
        current_leg_scale = r1_features['bottom_scale']
        target_level_name = r1_features['bottom_name']

    while True:
        random_modifier = random.randint(-2, 2)
        new_scale = int(current_leg_scale) + random_modifier
        if new_scale >= 0 and new_scale <= upper_bound: # Valid new leg_size
            break

    new_bot_name = assemble_robot_name( top_axis=r1_features['top_axis'],
                                        bottom_axis=r1_features['top_axis'],
                                        top_name=target_level_name,
                                        bottom_name=target_level_name,
                                        top_scale=new_scale,
                                        bottom_scale=new_scale,
                                        baselink_name=target_level_name)
    print(new_bot_name)
    return new_bot_name # Still only swaps level targeted

def create_mutate_joint_robot_string(robot_1, bank_size, level): # This util.ity creates a bank string with a mutated limb length. Same length for top and bot, pass requested level to mutate_links()
    r1_features = extract_robot_features(robot_1)

    if(level == 1):
        # mutate upper leg
        current_axis = r1_features['top_axis']

    if(level == 2):
        # mutate upper leg
        current_axis = r1_features['bottom_axis']

    new_axis = change_axis(current_axis)

    new_bot_name = assemble_robot_name( top_axis=new_axis,
                                        bottom_axis=new_axis,
                                        top_name=r1_features['top_name'],
                                        bottom_name=r1_features['top_name'],
                                        top_scale=r1_features['top_scale'],
                                        bottom_scale=r1_features['top_scale'],
                                        baselink_name=r1_features['top_name'])

    print(new_bot_name)
    return new_bot_name

def create_mutate_name_robot_string(robot_1, bank_size, level, robot_names): # This util.ity creates a bank string with a mutated limb length. Same length for top and bot, pass requested level to mutate_links()
    if(len(robot_names) == 1):
        return None # Cannot mutate names with only one name
    
    r1_features = extract_robot_features(robot_1)

    if(level == 1):
        # mutate upper leg
        current_name = r1_features['top_name']

    if(level == 2):
        # mutate upper leg
        current_name = r1_features['bottom_name']

    new_scale = random.choice(range(10))

    new_name = current_name
    while new_name == current_name:
        new_name = random.choice(robot_names) # Select a new name

    new_bot_name = assemble_robot_name( top_axis=r1_features['top_axis'],
                                        bottom_axis=r1_features['top_axis'],
                                        top_name=new_name,
                                        bottom_name=new_name,
                                        top_scale=new_scale,
                                        bottom_scale=new_scale,
                                        baselink_name=new_name)

    print(new_bot_name)
    return new_bot_name

def create_mutate_baselink_robot_string(robot_1, bank_size, robot_names): # This util.ity creates a bank string with a mutated limb length. Same length for top and bot, pass requested level to mutate_links()
    if(len(robot_names) == 1):
        return None # Cannot mutate names with only one name
    
    r1_features = extract_robot_features(robot_1)

    new_name = r1_features['base_link_name']
    while new_name == r1_features['base_link_name']:
        new_name = random.choice(robot_names) # Select a new name

    new_bot_name = assemble_robot_name( top_axis='x',
                                        bottom_axis='x',
                                        top_name=new_name,
                                        bottom_name=new_name,
                                        top_scale=0,
                                        bottom_scale=0,
                                        baselink_name=new_name)

    print(new_bot_name)
    return new_bot_name

def find_nn_path(base_directory):
    # Construct the path to the 'runs' directory
    runs_dir = os.path.join(base_directory, 'runs')
    
    try:
        # List directories in 'runs'
        subdirectories = next(os.walk(runs_dir))[1]  # Get subdirectories only
        if subdirectories:
            # Sort subdirectories alphabetically and pick the last one
            subdirectories.sort()
            last_subdirectory = subdirectories[-1]  # Last alphabetically
            # Construct the path to this subdirectory
            unique_folder_path = os.path.join(runs_dir, last_subdirectory)
            # Construct the path to the 'nn' directory
            nn_path = os.path.join(unique_folder_path, 'nn')
            return nn_path
        else:
            print("No subdirectories found in 'runs'.")
    except Exception as e:
        print(f"An error occurred: {e}")

def log_information(gen, robots, archive):
    logging.info("Completed generation: " + str(gen))

    logging.info("List of active offspring:")
    for robot in robots.values():
        logging.info(f"Model: {robot.model_dir}  Reward: {robot.reward}  Gen: {robot.generation}")
    
    logging.info("\nList of archived offspring:")
    for robot in archive.values():
        logging.info(f"Model: {robot.model_dir}  Reward: {robot.reward}  Gen: {robot.generation}")
    logging.info("")  # For an extra newline for readability

def log_print(text):
    logging.info(text)