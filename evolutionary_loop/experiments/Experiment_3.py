import os
extra_configs={ # glaobal
    "task.env.terrain.terrainType":"plane"
    # "task.env.randomCommandVelocityRanges.linear_x":[-0.5,0.5],
    # "task.env.randomCommandVelocityRanges.linear_y":[-0.5,0.5],
    # "task.env.randomCommandVelocityRanges.yaw":[-1,1]
}

ENTRY_POINT = os.path.abspath(f"{os.path.dirname(__file__)}/../../legged_env/envs/train.py")

HEAD_PLAY = f"python {ENTRY_POINT} task=RobotDog test=True num_envs=2 "
HEAD_TRAIN= f"python {ENTRY_POINT} task=RobotDog headless=True max_iterations=250 "
HEAD_TEST_TRAIN= f"python {ENTRY_POINT} task=RobotDog headless=True train.params.config.save_frequency=1 train.params.config.save_best_after=2 max_iterations=2 "

base_directory = "../../RobotsMakingRobots/evolutionary_loop/Experiment_3/Offspring"
# base_directory = "../assets/URDFsForBoxi"

experiment_name = "Experiment_3" # This should be changed based on the experiment to reference the correct folder. Bot_name strategy will need to be adjusted if multiple bots exist in the bank
robot_names = ['AntBot', 'BreadBot', 'BugBot1', 'BugBot2', 'BugBot3', 'BugBot4', 'BugBot5', 'CanBot', 'ChameleonBot', 'CheetahBot2', 'CornBot', 'CricketBot', 'FrogBot', 'IguanaBot', 'LizardBot', 'PlatyBot', 'RobotDog', 'SallyBot', 'ScorpoBot', 'ShoeBot'] # Add all the robot_names for Experiment 3 into this folder