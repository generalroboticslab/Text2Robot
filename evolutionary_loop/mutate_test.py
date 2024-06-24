from init_population import init
import os
from util import *
from swapping import *
from gen_config_file import g_config_file

max_generations = 30
gen = 0

bot_name = "RobotDog"
# For now assume that a folder with the same name as bot exists, and contains a subfolder 'URDF_bank' with all seeded solutions

bank_size = 30 # assume equal num of each joint type

# init(bot_name, bank_size)
# Train : iterate through each Offspring, if it contains a textfile named reward, do not retrain, else retrain and write max_reward to reward.txt

folder_path = os.path.join(bot_name, "Offspring")

foldernames = os.listdir(folder_path)

for filename in foldernames:
    # Mutate link with a 30% chance
    path = folder_path + "/" + filename

    robot_1 = filename
    print(robot_1)

    r1_features = extract_robot_features(robot_1)
    random_level = 1

    upper_bound = (bank_size / 3) - 1
    if(random_level == 1):
        # mutate upper leg
        current_leg_scale = r1_features['top_scale']

        while True:
            random_modifier = random.randint(-2, 2)
            new_top_scale = current_leg_scale + random_modifier
            if new_top_scale >= 0 and new_top_scale <= upper_bound: # Valid new leg_size
                break
        
        robot_2 = f"{bot_name}_top-y-2_bottom-y-2" # Still only swaps level targeted

    elif(random_level == 2):
        # mutate lower leg
        current_leg_scale = r1_features['bottom_scale']

        while True:
            random_modifier = random.randint(-2, 2)
            new_bottom_scale = current_leg_scale + random_modifier
            if new_bottom_scale >= 0 and new_bottom_scale <= upper_bound: # Valid new leg_size
                break
        
        print("New Scale: " + str(new_bottom_scale))
        robot_2 = f"{bot_name}_top-{r1_features['top_axis']}-{new_bottom_scale}_bottom-{r1_features['top_axis']}-{new_bottom_scale}" # Still only swaps level targeted

    # print("Mutate Link    R1: " + robot_1 + "  R2: " + robot_2 + "  Level: " + str(random_level))
    # swap = swap_legs_from_urdf(bot_name, robot_1, robot_2, folder_path, True)
    # swap.mutate_links(dest_path=folder_path, level=random_level)

    print("Swap Link    R1: " + robot_1 + "  R2: " + robot_2 + "  Level: " + str(random_level))
    swap = swap_legs_from_urdf(bot_name, robot_1, robot_2, folder_path, False)
    swap.swap_links(dest_path=folder_path, level=random_level)

g_config_file(bot_name+'/Offspring') 














