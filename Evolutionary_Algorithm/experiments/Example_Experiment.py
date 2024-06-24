import os
extra_configs={ # global
    "task.env.randomCommandVelocityRanges.linear_x":[-0.5,0.5],  # Here you can pass command line overrides customized to a particular experiment
    "task.env.randomCommandVelocityRanges.linear_y":[-0.5,0.5],
    "task.env.randomCommandVelocityRanges.yaw":[-1,1]
}
ENTRY_POINT = os.path.abspath(f"{os.path.dirname(__file__)}/../../legged_env/envs/train.py")

HEAD_PLAY = f"python {ENTRY_POINT} task=RobotDog test=True num_envs=2 "
HEAD_TRAIN= f"python {ENTRY_POINT} task=RobotDog headless=True max_iterations=250 "

experiment_name = "Example_Experiment" # This should be changed based on the experiment to reference the correct folder. Bot_name strategy will need to be adjusted if multiple bots exist in the bank
robot_names = ['ExampleBot'] # Array of all robot names in the provided URDF_Bank

