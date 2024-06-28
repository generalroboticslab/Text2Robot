# Text2Robot

Ryan Ringel,
[Zachary Charlick](https://zacharycharlick.com),
Jiaxun Liu,
Boxi Xia,
[Boyuan Chen](http://boyuanchen.com/)
<br>
Duke University
<br>

### [Project Website](http://www.generalroboticslab.com/Text2Robot) | [Video](https://youtu.be/Cwq7G6OUeGg) | [Paper](http://arxiv.org)

## Overview
This repo contains the Fusion360 and Python implementation for paper "Text2Robot." Our pipeline automatically converts a text prompt to a quadrupedal robot. We utilize a state of the art text to mesh generative model as initialization for our pipeline, and convert the static mesh to a kinetic robot model. We evolve the robots control and morphology simultaneously using our evolutionary algorithm.

<p align="center">
    <img src="figures/teaser.gif" alt="teaser" style="width:50%;">
</p>

## Citation

If you find our paper or codebase helpful, please consider citing:

```
@article{standincitation,
  title={Text2Robot},
  author={Ringel, Ryan and Charlick, Zachary and Liu, Jiaxun and Xia, Boxi and Chen, Boyuan},
  journal={arXiv preprint arXiv:STANDIN},
  year={2024}
}
```

## Content

- [Installation](#installation)
- [Text2Mesh](#text2mesh)
- [Mesh2CAD](#mesh2cad)
- [CAD2URDF](#cad2urdf)
- [Evolutionary Loop](#evolutionary-loop)
- [Sim2Real](#sim2real)

## Project Structure
```
├── conda_env_py38.yaml                     # Conda enviornment for isaacgym
├── Evolutionary_Algorithm
│   ├── driver.py                           # Evolutionary algo driver
│   ├── Example_Frog_Experiment             # Example Experiment URDF_Bank and directory
│   ├── experiments                         # Customize command line overrides for your experiment
│   ├── extract_rewards_from_tensorboard_file.py
│   ├── gen_config_file.py
│   ├── greatest_reward.py
│   ├── init_population.py
│   ├── __init__.py
│   ├── output
│   ├── __pycache__
│   ├── swapping.py
│   ├── train_util.py
│   └── util.py
├── Fusion360_Scripts                               
│   ├── Install_Packages                   # Install necessary python packages for Fusion360
│   └── Wrapper                            # Mesh to Robot Model(s) w/ geometric slicing
├── __init__.py
├── legged_env
│   ├── assets
│   ├── envs                               # Navigate here for visualization & sim2real
│   ├── __init__.py
│   ├── __pycache__
│   └── README.md
├── README.md
├── Sim2Real                               # Sim2Real receiver for RaspPi
│   └── receiver.py
└── STL_Files                              # Example Text-to-mesh and modular components
    ├── Electronics_Modules
    └── Example_Meshes
```
## Installation

The installation has been tested on Ubuntu 22.04.4 LTS with CUDA 12.3. The experiments are performed on several different servers with either PNY RTX A6000, NVIDIA GeForce RTX 3090, or NVIDIA A100 PCIe GPUs.

The conda yaml file `conda_env_py38.yaml` can be found in the base folder of the repo

```bash
alias conda="micromamba"

# Create environment
conda env create --file conda_env_py38.yaml -y

# Activate the environment
conda activate py38

# Export library path
export LD_LIBRARY_PATH=${CONDA_PREFIX}/lib
```

For Fusion360 geometric slicing installation, see section [Mesh2CAD](#mesh2cad)

## Text2Mesh
We use the Meshy website https://www.meshy.ai/ to generate STL meshes from text prompts. STL's used in our experiments are provided in `STL_Files/Example_Meshes`. 

![MeshyHomeScreen](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/0fa40918-f86d-4850-900c-a0971a081f2c)

Additional meshes can be generated using Meshy, although they are not guaranteed to work with the provided slicing script. Include the keywords "quadrupedal walking robot" in the text prompt for best results.

![MeshyText](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/72d1d86f-b40e-47d9-823a-877393302f4b)

## Mesh2CAD

STL meshes can be converted to a fusion360 assembly using the provided sliceBody script. First open Fusion360, and preprocess the generated or downloaded mesh desired. Installation instructions can be found at https://www.autodesk.com/campaigns/education/fusion-360. Insert the mesh into a fusion360 document, and use the convert mesh operation to convert the mesh to a brepboy. Organic mesh conversion is enabled with the fusion design extension.

![InsertMesh](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/6260c319-2e28-4328-9282-bf41ddf6e99f)
![ConvertMesh](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/752ee300-753c-41fc-bc06-1f0e3ca92f7b)

To slice the brepbody, add both python scripts in the 'Fusion360_Scripts' folder into Fusion360. To link an add-in in Fusion360, click on the green plus, and navigate to and select the Wrapper and Install_Packages folders.

![AddScripts](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/761fe4d8-8e56-4d94-aae2-a468c2e5c465)

Prior to running the Wrapper script, add "Polyethylene Low Density" material to favorites by navigating to the fusion material library plastics folder, right clicking on polyethylene low density, and adding to favorites. 

![AddMaterial](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/abde0655-7a6f-4fc9-a84d-02c716c1f1b0)

Run Install_Packages to install necessary python libraries to the same file path as the wrapper script. Running the Wrapper script will convert the preprocessed brepbody to a robot assembly. If the mesh does not properly slice, adjusting the steps in the slicebyDX function of slicebody can adjust the location of shoulder slices.

![AdjustSteps](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/22e2407d-51d0-4c4e-9bb2-3220d6cb2993)


## CAD2URDF

Uncommenting the URDF exporter function in wrapper will export the generated robot to a URDF. Uncommenting the loop will create 30 variations of the generated robot, and export all as URDF's. This may take up to 15 minutes.

![URDFExporter](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/2199d1c2-33e8-4c08-9eb5-a24ddb90e0e1)

## Evolutionary Loop

To further evolve robots using our evolutionary algorithm, create an experiment directory similar to our provided example: `Evolutionary_Algorithm/Example_Experiment`. If you wish to make custom changes to the training parameters, you can create a configuration file for the experiment in the `Evolutionary_Algorithm/experiments` directory, modeling it after our example. This directory should contain a folder `URDF_Bank`, which will hold the entire gene pool. We recommend using at least 5 prompts for a total of 150 robot models. If you have fewer, you will need to modify `Evolutionary_Algorithm/init_population` to initialize a smaller first generation. You can include as many robots as you would like!

You can run `driver.py` out of the box to train our example Frog experiment! Or, to use your own models, under the `Evolutionary_Algorithm/experiments` folder, duplicate and rename `Example_Frog_Experiment.py`. Change the `robot_names` array to include the names of each prompt in the bank. Ensure the naming convention for your robots is correct. For example, if you have a bank of just one prompt 'Frog' (with 30 total models), each of the 30 models should follow the naming convention: `Frog_Frog_top-z-9_Frog_bottom-z-9.urdf` with 'z' and '9' replaced by the three axis orientations `[x, y, z]`, and all possible limb scales `0-9`.

Lines 20-25 of `driver.py` can be edited to change the number of generations of evolution, as well as preferences for energy efficiency, velocity tracking accuracy, or performance on rough terrain.

```bash
max_generations = 55
inform_based_on_energy = False
inform_based_on_velocity = False
rough_terrain = False
```

You will also need to adjust the GPU configuration settings on `driver.py` line `#67` to match your system properties.

After setting up the experiment, navigate to the `Evolutionary_Algorithm` folder, and run `driver.py`. `driver.py` can be updated to change the max generations of evolution, rough vs flat terrain, and if there should be an additional user specified preference of velocity tracking accuracy, or energy efficiency.

After running the experiment, you can visualize any of the evolved robots and their walking policies. Navigate to `legged_env/envs` and run:
```bash
bash run.sh example -pk
```
to visualize an example checkpoint of an evolved frog. To visualize your own bot, create a new entry in `exp.sh` modeled after `example`. Here you can specify the path to the robot urdf, and the path to the checkpoint file. -pk specifies playback mode, with keyboard input enabled. This will allow you to control the velocity of your robot using (ijkl) as arrow keys and (u and o) to control yaw (rotational velocity). For a full list of playback options, check out `run.sh`. If you run into unexpected errors, make sure you have the conda environment activated!

## Sim2Real

For Sim2Real, download the Sim2Real_Receiver/receiver.py folder on to your robots RaspberryPi. You will also need to initialize the servo python control library found at: https://github.com/ethanlipson/PyLX-16A

Running the receiver.py python script will listen for the UDP message packs that are transmitted when a robot is being played in simulation. You will need to match the target_url of the data receiver and data publisher. The data publisher url can be modified using a command line override to issacgym "task.env.dataPublisher.target_url," and can be specified in the bash profile for playback in `legged_envs/envs/exp.sh`. We provide an example in this file.

```bash
example(){
    base
    task=RobotDog
    PLAY_ARGS+=(
        num_envs=1
        checkpoint=assets/checkpoints/example_frog.pth
    )
    BASE_ARGS+=(
    # task.env.terrain.terrainType=plane
    ++task.env.urdfAsset.root="assets/urdf/example_frog"
    task.env.urdfAsset.file="frog.urdf"
    task.env.randomCommandVelocityRanges.linear_x=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.linear_y=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.yaw=[-1,1]

    task.env.dataPublisher.enable=true
    task.env.dataPublisher.target_url=udp://10.172.14.96:9870
    )
}
```

## License

This repository is released under the STAND IN license.  STAND IN
