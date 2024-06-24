import os
extra_configs={ # glaobal
    "task.env.terrain.terrainType":"plane",
    "task.env.urdfAsset.collision_filter":"1",
    "task.env.urdfAsset.AssetOptions.collapse_fixed_joints":"false"
    # "task.env.urdfAsset.root=../evolutionary_loop/assets/robogrammar_bank
    # "task.env.urdfAsset.file="robot_1350/robot.urdf" # example
}

ENTRY_POINT = os.path.abspath(f"{os.path.dirname(__file__)}/../../legged_env/envs/train.py")

HEAD_PLAY = f"python {ENTRY_POINT} task=RobotDog test=True num_envs=2 "
HEAD_TRAIN= f"python {ENTRY_POINT} task=RobotDog headless=True max_iterations=250 "
HEAD_TEST_TRAIN= f"python {ENTRY_POINT} task=RobotDog headless=True train.params.config.save_frequency=1 train.params.config.save_best_after=2 max_iterations=2 "

base_directory = "../../RobotsMakingRobots/evolutionary_loop/Experiment_3/Offspring"
# base_directory = "../assets/URDFsForBoxi"

experiment_name = "Experiment_4" # This should be changed based on the experiment to reference the correct folder. Bot_name strategy will need to be adjusted if multiple bots exist in the bank