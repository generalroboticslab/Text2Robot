import hydra
from omegaconf import DictConfig, OmegaConf
import isaacgym
import isaacgymenvs
import torch
import numpy as np

import sys
import os
sys.path.append(isaacgymenvs.__path__[0])  # fix isaacgymenvs imports
sys.path.append(os.path.abspath(__file__ + "/../.."))  # fix envs imports

from isaacgymenvs.tasks import isaacgym_task_map
from envs.tasks.legged_terrain import LeggedTerrain
isaacgym_task_map["A1Terrain"] = LeggedTerrain
isaacgym_task_map["AnymalTerrain"] = LeggedTerrain
isaacgym_task_map["RobotDog"] = LeggedTerrain
isaacgym_task_map["Biped"] = LeggedTerrain

# using np.arange [star,end), step, round to 15 decimals
OmegaConf.register_new_resolver(
    "arange", lambda start, stop, step: list(np.round(np.arange(start, stop, step), 15)), replace=True
)

# using np.arange [star,end), step, round to 15 decimals
OmegaConf.register_new_resolver(
    "linspace", lambda start, stop, num: list(np.round(np.linspace(start, stop, num), 15)), replace=True
)



@hydra.main(version_base="1.1", config_name="config", config_path="./cfg")
def launch_rlg_hydra(cfg: DictConfig):
    # print(OmegaConf.to_yaml(cfg))
    envs = isaacgymenvs.make(
        cfg.seed,
        cfg.task_name,
        cfg.task.env.numEnvs,
        cfg.sim_device,
        cfg.rl_device,
        cfg.graphics_device_id,
        cfg.headless,
        cfg.multi_gpu,
        cfg.capture_video,
        cfg.force_render,
        cfg
    )

    envs.reset()
    
    actions = torch.zeros(envs.num_envs, envs.num_actions, device=envs.device)

    for k in range(1000000):
        envs.step(actions)


if __name__ == "__main__":
    launch_rlg_hydra()