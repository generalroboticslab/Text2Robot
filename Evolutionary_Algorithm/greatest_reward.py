import os

# Find the checkpoint file with the greatest reward. Return the filename and the reward.

# Parameters
# folder_path: The path to the folder created by the training

def find_greatest_reward(folder_path):
    # print("Checkpoint target: " + str(folder_path))
    nn_folder_path = folder_path

    max_reward = -10000000
    max_file = ""

    if os.path.isdir(nn_folder_path):
        filenames = os.listdir(nn_folder_path)
        for filename in filenames:
            if "rew_" in filename and "_." not in filename:
                reward = float(filename.split("rew_", 1)[1].split(".pth", 1)[0])
                if reward > max_reward:
                    max_reward = reward
                    max_file = filename
    else:
        print("Subfolder 'nn' does not exist: " + folder_path)

    return max_reward, max_file

