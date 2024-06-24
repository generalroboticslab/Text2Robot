import time
import sys
import random
import hydra
from omegaconf import DictConfig, OmegaConf
import os
@hydra.main(version_base="1.1", config_name="config", config_path="./cfg")
def launch_rlg_hydra(cfg: DictConfig):
    print(OmegaConf.to_yaml(cfg))
    # path= os.path.abspath('./generated_config.yaml')
    # with open(path, 'w') as f:
    #     f.write(OmegaConf.to_yaml(cfg))
    #     print(f"saved to {path}")
    
    
if __name__=="__main__":
    launch_rlg_hydra()
    # with open("./log.txt",'w'):
    #     # print("starting an exp")
    #     print(time.time())
    #     time.sleep(random.random())
    #     # print("finish an exp")
    #     sys.exit()