import os
extra_configs={ # glaobal
    "task.env.terrain.terrainType":"plane",
    "task.env.randomCommandVelocityRanges.linear_x":[-0.5,0.5],
    "task.env.randomCommandVelocityRanges.linear_y":[-0.5,0.5],
    "task.env.randomCommandVelocityRanges.yaw":[-1,1]
}
ENTRY_POINT = os.path.abspath(f"{os.path.dirname(__file__)}/../../legged_env/envs/train.py")

HEAD_PLAY = f"python {ENTRY_POINT} task=RobotDog test=True num_envs=2 "
HEAD_TRAIN= f"python {ENTRY_POINT} task=RobotDog headless=True max_iterations=250 "

base_directory = "../../RobotsMakingRobots/evolutionary_loop/Experiment_1/Offspring"
# base_directory = "../assets/URDFsForBoxi"

experiment_name = "Experiment_1" # This should be changed based on the experiment to reference the correct folder. Bot_name strategy will need to be adjusted if multiple bots exist in the bank
robot_names = ['RobotDog'] # Array of all robot names in the bank
