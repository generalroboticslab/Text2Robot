import pathlib
import os
import re

def correct_directory_names(parent_directory):
    parent_path = pathlib.Path(parent_directory)
    
    # Iterate over each entry in the parent directory
    for folder in parent_path.iterdir():
        if folder.is_dir():
            # Extract the original folder name
            original_name = folder.name
            # Construct the new folder name by fixing the naming convention
            parts = re.split(r'[-_]', original_name)
            print(parts)

            if len(parts) == 9: 
                bot_name = parts[0]
                axis = parts[3]
                scale = parts[4]

                new_name = f"{bot_name}_{bot_name}_top-{axis}-{scale}_{bot_name}_bottom-{axis}-{scale}"
                print(new_name)
                new_folder_path = folder.parent / new_name
                print(new_folder_path)
                
                # Rename the folder
                folder.rename(new_folder_path)
                
                # Rename files within the directory
                for file in new_folder_path.iterdir():
                    if file.suffix in ['.yaml', '.urdf']:
                        old_file_name = file.stem
                        new_file_name = new_name + file.suffix
                        file.rename(new_folder_path / new_file_name)

                print(f"Renamed '{original_name}' to '{new_name}'")
            else:
                print(f"Skipped '{original_name}' due to unexpected format")


parent_directory = '/home/grl/Documents/RobotsMakingRobots/evolutionary_loop/Experiment_5/URDF_Bank' 
correct_directory_names(parent_directory)