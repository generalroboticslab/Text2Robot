import pathlib
import os
import contextlib
import yaml
import yourdfpy
import numpy as np

def create_nested_dict(key, value):
    """Converts a dot-separated key and value into a nested dictionary."""
    keys = key.split('.')
    current_dict = {keys[-1]:value}
    for key_part in reversed(keys[0:-1]):
        current_dict = {key_part: current_dict}
    return current_dict


def flatten_dict(nested_dict, prefix=None):
    """Flattens a nested dictionary, keys are combined with dot separators."""
    flattened_dict = {}
    for key, value in nested_dict.items():
        new_key = key if prefix is None else prefix + "." + key
        if isinstance(value, dict):
            flattened_dict.update(flatten_dict(value, new_key))
        else:
            flattened_dict[new_key] = value
    return flattened_dict

def get_subdirectory_configs(sub_directory_path:pathlib.Path,extra_configs, yaml_supplied=False, rough_terrain=False):
    print(sub_directory_path)
    urdf_path = next(sub_directory_path.rglob("*.urdf"))
    urdf_dir, urdf_base_path = os.path.split(os.path.abspath(urdf_path))
    if rough_terrain:
         config = {
            'task.env.urdfAsset.file':f"\'{urdf_base_path}\'",
            '++task.env.urdfAsset.root':f"\'{urdf_dir}\'", 
            'task.env.terrain.terrainType': "heightfield",
            'task.env.terrain.difficultySale':0.2,
            'task.env.terrain.curriculum': "true",
            'task.env.terrain.terrainProportions': [1,1,1,0,0,0,1,1,0,0],
            '++task.env.urdfAsset.AssetOptions.vhacd_enabled': "true",
            '++task.env.urdfAsset.AssetOptions.vhacd_params.max_convex_hulls': 3,
            '++task.env.urdfAsset.AssetOptions.vhacd_params.max_num_vertices_per_ch': 16
        }
    else:
        config = {
            'task.env.urdfAsset.file':f"\'{urdf_base_path}\'",
            '++task.env.urdfAsset.root':f"\'{urdf_dir}\'"
        }

    if yaml_supplied:
        extra_yaml = list(sub_directory_path.rglob("*.yaml"))
        if any(extra_yaml):
            with open(extra_yaml[0], 'r') as file:
                data = yaml.safe_load(file)        
            config.update(flatten_dict(data))

    config.update(extra_configs)
    
    return config

import contextlib


# https://stackoverflow.com/questions/75048986/way-to-temporarily-change-the-directory-in-python-to-execute-code-without-affect
@contextlib.contextmanager
def new_cd(x):
    d = os.getcwd()
    # This could raise an exception, but it's probably
    # best to let it propagate and let the caller
    # deal with it, since they requested x
    os.chdir(x)

    try:
        yield

    finally:
        # This could also raise an exception, but you *really*
        # aren't equipped to figure out what went wrong if the
        # old working directory can't be restored.
        os.chdir(d)





def normalized_velocity_command(
    urdf: yourdfpy.URDF,
    standard_unit: float = 0.5,  # Default unit for normalization
    base_command_range: dict = {  # Use a default dict here
        "linear_x": [-0.5, 0.5],
        "linear_y": [-0.5, 0.5],
        "yaw": [-1, 1],
    },
) -> str:
    """
    Calculates normalized velocity command ranges for a given URDF model.
    
    Args:
        urdf: The URDF model object.
        standard_unit: The reference unit for normalization.
        base_command_range: The base command ranges to be scaled.
    
    Returns:
        A string containing the formatted velocity commands for task environment setup.

    example:
        urdf_path = "path_to_URDF" 
        urdf = yourdfpy.URDF.load(urdf_path)
        commands = normalized_velocity_command(urdf)

    """
    
    # Get bounding box extents
    extents = urdf.scene.bounding_box.extents
    
    # Calculate scaling factor based on standard unit and the largest extent
    max_extent = max(extents)
    scale = np.sqrt(max_extent / standard_unit) 

    # Scale the base command ranges
    commands = {}
    for axis, base_range in base_command_range.items():
        commands[axis] = np.array(base_range) * scale

    # Create the formatted command string using f-strings (more concise)
    return (
        f"task.env.randomCommandVelocityRanges.linear_x=[{commands['linear_x'][0]:.3f},{commands['linear_x'][1]:.3f}] "
        f"task.env.randomCommandVelocityRanges.linear_y=[{commands['linear_y'][0]:.3f},{commands['linear_y'][1]:.3f}] "
        f"task.env.randomCommandVelocityRanges.yaw=[{commands['yaw'][0]:.3f},{commands['yaw'][1]:.3f}]")