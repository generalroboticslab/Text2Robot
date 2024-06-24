import os

# Parameters
# folder_path: The path to the folder created by the training
# right now this is under outputs/TaskName/False/Date  (False for training)

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


# reward, file = find_greatest_reward("/home/grl/Documents/legged/outputs/RobotDog/False/2024-05-07_09-44-27/runs/RobotDog_07-09-44-28/nn")

# print(f"Max reward: {reward} from file: {file}")

