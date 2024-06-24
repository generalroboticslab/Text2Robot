# #!/bin/bash



biped_hang(){
    base
    task=Biped
    PLAY_ARGS+=(
        
    )
    BASE_ARGS+=(
        
    )
}


biped_o1(){
    # remove heightmap
    base
    task=Biped
    TRAIN_ARGS+=(
        train.params.config.max_epochs=5000
        headless=false
        # checkpoint=outputs/Biped/False/20240622_002720/runs/Biped_22-00-27-20/nn/Biped.pth
        # checkpoint=outputs/Biped/False/20240621_165729/runs/Biped_21-16-57-29/nn/Biped.pth # gravity=[0,0,-2] #1
        # checkpoint=outputs/Biped/False/20240621_171127/runs/Biped_21-17-11-27/nn/last_Biped_ep_2500_rew_26.29728.pth # gravity=[0,0,-4]
    )
    PLAY_ARGS+=(
        checkpoint=outputs/Biped/False/20240622_192733/runs/Biped_22-19-27-33/nn/Biped.pth
        # checkpoint=outputs/Biped/False/20240622_002720/runs/Biped_22-00-27-20/nn/Biped.pth
        # checkpoint=outputs/Biped/False/20240621_235328/runs/Biped_21-23-53-28/nn/Biped.pth
        # checkpoint=outputs/Biped/False/20240621_181456/runs/Biped_21-18-14-56/nn/Biped.pth
        # checkpoint=outputs/Biped/False/20240620_181243/runs/Biped_20-18-12-44/nn/Biped.pth
        num_envs=4
        task.env.dataPublisher.enable=true

        task.sim.dt=0.0025
        ++task.env.renderFPS=100

        # ++task.env.renderFPS=50
        task.env.learn.episodeLength_s=999
        # "task.env.dataPublisher.target_url=udp://10.172.14.96:9870"

        # task.env.terrain.terrainType=plane


    )
    BASE_ARGS+=(
        task.env.urdfAsset.file=urdf/biped/v6_cylinder_leg.urdf
        task.env.urdfAsset.AssetOptions.replace_cylinder_with_capsule=true

        # contact in obs does not work
        # "task.env.observationNames=[linearVelocity,angularVelocity,projectedGravity,commands,dofPosition,dofVelocity,actions,contact]"
        
        "task.env.observationNames=[linearVelocity,angularVelocity,projectedGravity,commands,dofPosition,dofVelocity,actions]"

        task.sim.physx.num_position_iterations=2 # reduced for faster training # works!


        # dynamics matching
        ++task.env.assetDofProperties.friction=0.008
        ++task.env.assetDofProperties.armature=0.2

        task.env.randomize.push.enable=true
        task.env.randomize.friction.enable=true
        task.env.learn.addNoise=true
        ++task.env.baseHeightOffset=0.05

        ++task.env.defaultJointPositions=[0.000,0.175,0.000,0.387,-0.213,0.000,-0.175,0.000,-0.387,0.213]

        ++task.env.assetDofProperties.velocity=10

        # task.sim.gravity=[0,0,-2] #1
        # task.sim.gravity=[0,0,-4]
        # task.sim.gravity=[0,0,-9.81]


        task.env.randomCommandVelocityRanges.linear_x=[0.5,0.5]
        task.env.randomCommandVelocityRanges.linear_y=[0,0]
        task.env.randomCommandVelocityRanges.yaw=[0,0]

        task.env.terrain.terrainType=plane
        
        # task.env.terrain.terrainType=heightfield
        # task.env.terrain.difficultySale=0.2
        # task.env.terrain.curriculum=true
        # task.env.terrain.terrainProportions=[1,1,1,0,0,0,1,1,0,0]
    )
}

biped(){
    # bash run.sh biped -p
    base
    task=Biped
    TRAIN_ARGS+=(
        checkpoint=outputs/Biped/False/20240620_171044/runs/Biped_20-17-10-44/nn/Biped.pth
        # checkpoint=outputs/Biped/False/20240617_095803/runs/Biped_17-09-58-03/nn/Biped.pth
        # checkpoint=outputs/Biped/False/20240614_111610/runs/Biped_14-11-16-10/nn/last_Biped_ep_2500_rew_23.220852.pth  # good
        train.params.config.max_epochs=5000
        headless=false

        task.env.randomCommandVelocityRanges.linear_x=[0.5,0.5]
        task.env.randomCommandVelocityRanges.linear_y=[0,0]
        task.env.randomCommandVelocityRanges.yaw=[0,0]

    )
    PLAY_ARGS+=(
        # checkpoint=outputs/Biped/False/20240619_191840/runs/Biped_19-19-18-40/nn/last_Biped_ep_5000_rew_11.690597.pth # train on terrain
        # checkpoint=outputs/Biped/False/20240619_183729/runs/Biped_19-18-37-29/nn/Biped.pth
        # checkpoint=outputs/Biped/False/20240617_095803/runs/Biped_17-09-58-03/nn/Biped.pth
        # checkpoint=outputs/Biped/False/20240614_111610/runs/Biped_14-11-16-10/nn/last_Biped_ep_2500_rew_23.220852.pth  # good
        # checkpoint=outputs/Biped/False/20240614_130335/runs/Biped_14-13-03-35/nn/last_Biped_ep_1500_rew_21.036402.pth
        # checkpoint=outputs/Biped/False/20240614_142316/runs/Biped_14-14-23-16/nn/last_Biped_ep_3000_rew_22.302668.pth
        ++task.env.renderFPS=50
        task.env.learn.episodeLength_s=999

        task.env.randomCommandVelocityRanges.linear_x=[0.5,0.5]
        task.env.randomCommandVelocityRanges.linear_y=[0,0]
        task.env.randomCommandVelocityRanges.yaw=[0,0]

        task.env.dataPublisher.enable=true
        # ++task.env.urdfAsset.AssetOptions.vhacd_enabled=true



        task.env.enableDebugVis=True
        num_envs=1
        # task.env.terrain.terrainType=plane
        # # task.env.terrain.terrainType=heightfield
        # # task.env.terrain.terrainType=trimesh
        task.env.terrain.numLevels=2
        task.env.terrain.numTerrains=5
        # task.env.terrain.curriculum=False

        # task.env.randomCommandVelocityRanges.linear_x=[0,0]
        # task.env.randomCommandVelocityRanges.linear_y=[0,0]
        # task.env.randomCommandVelocityRanges.yaw=[0,0]
    )

    BASE_ARGS+=(
        ++task.env.defaultJointPositions=[0.000,0.175,0.000,0.387,-0.213,0.000,-0.175,0.000,-0.387,0.213]
        # ++task.env.urdfAsset.AssetOptions.override_inertia=true


        task.env.randomize.push.enable=false
        task.env.randomize.friction.enable=false
        task.env.learn.addNoise=false
        # ++task.env.baseHeightOffset=0.05

        # task.env.terrain.terrainType=plane

        ++task.env.baseHeightOffset=0.1
        task.env.terrain.terrainType=heightfield
        task.env.terrain.difficultySale=0.2
        task.env.terrain.curriculum=true
        task.env.terrain.terrainProportions=[1,1,1,0,0,0,1,1,0,0]
    )


}



shoe(){
    base
    task=RobotDog
    PLAY_ARGS+=(
        num_envs=1
        checkpoint=outputs/RobotDog/False/20240613_024701/runs/RobotDog_13-02-47-01/nn/RobotDog.pth
        # checkpoint=outputs/RobotDog/False/20240601_221336/runs/RobotDog_01-22-13-37/nn/RobotDog.pth
        # checkpoint=outputs/RobotDog/False/20240601_221336/runs/RobotDog_01-22-13-37/nn/last_RobotDog_ep_1500_rew_26.969398.pth
        # checkpoint=../evolutionary_loop/assets/ShoeBot_BugBot3_top-z-0_BreadBot_bottom-x-3/shoe.pth
        # task.env.dataPublisher.enable=true
        # task.env.learn.episodeLength_s=5

        # task.env.terrain.terrainType=trimesh


        task.env.terrain.numLevels=2
        task.env.terrain.numTerrains=5
        # task.env.terrain.curriculum=true
        num_envs=20
    )
    
    TRAIN_ARGS+=(
        headless=false
    )
    BASE_ARGS+=(

    task.env.terrain.terrainType=heightfield
    task.env.terrain.difficultySale=0.2
    task.env.terrain.curriculum=true
    task.env.terrain.terrainProportions=[1,1,1,0,0,0,1,1,0,0]
    ++task.env.urdfAsset.AssetOptions.vhacd_enabled=true
    ++task.env.urdfAsset.AssetOptions.vhacd_params.max_convex_hulls=3
    ++task.env.urdfAsset.AssetOptions.vhacd_params.max_num_vertices_per_ch=16

    # ++task.env.urdfAsset.AssetOptions.vhacd_params.convex_hull_approximation=false

    # task.env.terrain.terrainType=plane
    ++task.env.urdfAsset.root="../evolutionary_loop/assets"
    task.env.urdfAsset.file="ShoeBot_BugBot3_top-z-0_BreadBot_bottom-x-3/ShoeBot_BugBot3_top-z-0_BreadBot_bottom-x-3.urdf"
    task.env.randomCommandVelocityRanges.linear_x=[0.1,0.4]
    task.env.randomCommandVelocityRanges.linear_y=[0,0]
    task.env.randomCommandVelocityRanges.yaw=[0,0]
    )
}


frog(){
    base
    task=RobotDog
    PLAY_ARGS+=(
        num_envs=1
        # checkpoint=../evolutionary_loop/assets/FrogBot5_FrogBot1_top-z-0_FrogBot5_bottom-x-5/frogbot.pth
        checkpoint=outputs/RobotDog/False/20240604_155930/runs/RobotDog_04-15-59-30/nn/RobotDog.pth
        # task.env.dataPublisher.enable=true
        # task.env.learn.episodeLength_s=5
    )
    BASE_ARGS+=(
    # task.env.terrain.terrainType=plane
    ++task.env.urdfAsset.root="../evolutionary_loop/assets"
    task.env.urdfAsset.file="FrogBot5_FrogBot1_top-z-0_FrogBot5_bottom-x-5/FrogBot5_FrogBot1_top-z-0_FrogBot5_bottom-x-5.urdf"
    # task.env.randomCommandVelocityRanges.linear_x=[0.1,0.4]
    # task.env.randomCommandVelocityRanges.linear_y=[0,0]
    # task.env.randomCommandVelocityRanges.yaw=[0,0]

    ++task.env.urdfAsset.AssetOptions.vhacd_enabled=true
    ++task.env.urdfAsset.AssetOptions.vhacd_params.max_convex_hulls=5

    task.env.randomCommandVelocityRanges.linear_x=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.linear_y=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.yaw=[-1,1]
    )
}


cricket(){
    base
    task=RobotDog
    PLAY_ARGS+=(
        num_envs=1
        # checkpoint=../evolutionary_loop/assets/CricketBot_RobotDog_top-z-1_CanBot_bottom-x-0/cricket.pth
        checkpoint=outputs/RobotDog/False/20240603_173047/runs/RobotDog_03-17-30-47/nn/RobotDog.pth
        # task.env.dataPublisher.enable=true
        # task.env.learn.episodeLength_s=5
    )
    TRAIN_ARGS+=(
        # headless=false
    )
    BASE_ARGS+=(
    ++task.env.urdfAsset.AssetOptions.vhacd_enabled=true
    ++task.env.urdfAsset.AssetOptions.vhacd_params.max_convex_hulls=5

    # task.env.terrain.terrainType=plane
    ++task.env.urdfAsset.root="../evolutionary_loop/assets"
    task.env.urdfAsset.file="CricketBot_RobotDog_top-z-1_CanBot_bottom-x-0/CricketBot_RobotDog_top-z-1_CanBot_bottom-x-0.urdf"
    task.env.randomCommandVelocityRanges.linear_x=[0.1,0.4]
    task.env.randomCommandVelocityRanges.linear_y=[0,0]
    task.env.randomCommandVelocityRanges.yaw=[0,0]
    )
}



robogram(){
    base
    task=RobotDog
    PLAY_ARGS+=(
        
        # checkpoint=outputs/RobotDog/False/2024-05-23_09-50-54/runs/RobotDog_23-09-50-54/nn/RobotDog.pth
        checkpoint=outputs/RobotDog/False/2024-05-23_10-20-38/runs/RobotDog_23-10-20-39/nn/RobotDog.pth
        num_envs=2
        task.env.dataPublisher.enable=true
        pipeline="cpu"
        task.env.learn.episodeLength_s=5
        # task.env.randomCommandVelocityRanges.linear_x=[0.5,0.5]
        # task.env.randomCommandVelocityRanges.linear_y=[0.,0.]
        # task.env.randomCommandVelocityRanges.yaw=[1,1]
    )

    BASE_ARGS+=(
    task.env.urdfAsset.collision_filter=1
    task.env.urdfAsset.AssetOptions.collapse_fixed_joints=false
    ++task.env.urdfAsset.root=../evolutionary_loop/assets/robogrammar_bank
    task.env.urdfAsset.file="robot_1350/robot.urdf"
    task.env.terrain.terrainType=plane
    task.env.randomCommandVelocityRanges.linear_x=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.linear_y=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.yaw=[-1,1]
    )
}


bug(){
    base
    task=RobotDog
    PLAY_ARGS+=(
        num_envs=1
        # checkpoint=outputs/RobotDog/False/2024-05-27_11-40-19/runs/RobotDog_27-11-40-19/nn/RobotDog.pth
        checkpoint=outputs/RobotDog/False/2024-05-28_00-55-58/runs/RobotDog_28-00-55-58/nn/RobotDog.pth
        # task.env.dataPublisher.enable=true
        # task.env.learn.episodeLength_s=5
        # task.env.randomCommandVelocityRanges.linear_x=[0.5,0.5]
        # task.env.randomCommandVelocityRanges.linear_y=[0.,0.]
        # task.env.randomCommandVelocityRanges.yaw=[1,1]
    )
    BASE_ARGS+=(
    # task.env.terrain.terrainType=plane
    ++task.env.urdfAsset.root="../evolutionary_loop/assets"
    task.env.urdfAsset.file="BugBot4_BugBot3_top-z-0_BugBot2_bottom-x-0/BugBot4_BugBot3_top-z-0_BugBot2_bottom-x-0.urdf"
    task.env.randomCommandVelocityRanges.linear_x=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.linear_y=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.yaw=[-1,1]
    )
}



dog3kgv1(){
    base
    task=RobotDog
    PLAY_ARGS+=(

        checkpoint=/home/grl/repo/RobotsMakingRobots/legged_env/outputs/RobotDog/False/20240603_163954/runs/RobotDog_03-16-39-55/nn/RobotDog.pth
        
        num_envs=1
        task.env.dataPublisher.enable=true
        # task.env.learn.episodeLength_s=5
        # task.env.randomCommandVelocityRanges.linear_x=[0.5,0.5]
        # task.env.randomCommandVelocityRanges.linear_y=[0.,0.]
        # task.env.randomCommandVelocityRanges.yaw=[1,1]
    )
    TRAIN_ARGS+=(
        # headless=false
    )
    BASE_ARGS+=(
    task.env.urdfAsset.file="urdf/RobotDog/RobotDog3kg.urdf"
    task.env.terrain.terrainType=plane
    task.env.randomCommandVelocityRanges.linear_x=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.linear_y=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.yaw=[-1,1]
    )
}


dog3kg(){
    base
    task=RobotDog
    PLAY_ARGS+=(
        # checkpoint=../assets/checkpoints/RobotDog.pth
        # checkpoint=outputs/RobotDog/False/2024-05-14_15-20-17-dog3kg/runs/RobotDog_14-15-20-17/nn/RobotDog.pth
        # checkpoint=outputs/RobotDog/False/2024-05-14_23-33-01-dog3kg/runs/RobotDog_14-23-33-01/nn/RobotDog.pth
        # checkpoint=outputs/RobotDog/False/2024-05-15_17-58-21-dog3kg/runs/RobotDog_15-17-58-21/nn/RobotDog.pth
        checkpoint=outputs/RobotDog/False/2024-05-15_18-19-59/runs/RobotDog_15-18-19-59/nn/RobotDog.pth
        num_envs=1
        task.env.dataPublisher.enable=true
        pipeline="cpu"
    )
    BASE_ARGS+=(
    task.env.urdfAsset.file="urdf/RobotDog/RobotDog7kg.urdf"
    task.env.terrain.terrainType=plane
    task.env.randomCommandVelocityRanges.linear_x=[0.2,0.2]
    task.env.randomCommandVelocityRanges.linear_y=[0,0]
    task.env.randomCommandVelocityRanges.yaw=[0,0]
    )
}


dog(){
    base
    task=RobotDog
    PLAY_ARGS+=(
        checkpoint=../assets/checkpoints/RobotDog.pth
        num_envs=1
        # task.env.terrain.terrainType=plane
        # task.env.dataPublisher.enable=True
        # # headless (server)
        # headless=true
        # graphics_device_id=-1
    )
    TRAIN_ARGS+=(
        # headless=false
    )
    BASE_ARGS+=(
        task.env.terrain.terrainType=plane
        task.env.randomCommandVelocityRanges.linear_x=[0.2,0.2]
        task.env.randomCommandVelocityRanges.linear_y=[0,0]
        task.env.randomCommandVelocityRanges.yaw=[0,0]

        # "++task.env.defaultJointPositions='{joint_1_0: 0,joint_1_1: 0,joint_1_2: 0,joint_1_3: 0,joint_2_0: 0,joint_2_1: 0,joint_2_2: 0,joint_2_3: 0}'"
        # "++task.env.desiredJointPositions='{joint_1_0: 0,joint_1_1: 0,joint_1_2: 0,joint_1_3: 0,joint_2_0: 0,joint_2_1: 0,joint_2_2: 0,joint_2_3: 0}'"
        # ~task.env.defaultJointPositions
        # ~task.env.desiredJointPositions
        # "++task.env.defaultJointPositions.joint_1_0=0"
        # "++task.env.defaultJointPositions.joint_2_0=0"
        # "++task.env.defaultJointPositions.joint_1_1=0"
        # "++task.env.defaultJointPositions.joint_2_1=0"
        # "++task.env.defaultJointPositions.joint_1_2=0"
        # "++task.env.defaultJointPositions.joint_2_2=0"
        # "++task.env.defaultJointPositions.joint_1_3=0"
        # "++task.env.defaultJointPositions.joint_2_3=0"
        # "++task.env.desiredJointPositions.joint_1_0=0"
        # "++task.env.desiredJointPositions.joint_2_0=0"
        # "++task.env.desiredJointPositions.joint_1_1=0"
        # "++task.env.desiredJointPositions.joint_2_1=0"
        # "++task.env.desiredJointPositions.joint_1_2=0"
        # "++task.env.desiredJointPositions.joint_2_2=0"
        # "++task.env.desiredJointPositions.joint_1_3=0"
        # "++task.env.desiredJointPositions.joint_2_3=0"
    )
}

a1(){
    # bash run.sh a1Terrain -p
    base
    task=A1Terrain
    PLAY_ARGS+=(
        # checkpoint=outputs/A1Terrain/False/2024-05-15_23-54-50/runs/A1Terrain_15-23-54-50/nn/A1Terrain.pth
        checkpoint=outputs/A1Terrain/False/2024-05-16_00-50-46/runs/A1Terrain_16-00-50-46/nn/A1Terrain.pth
        task.env.dataPublisher.enable=true
        num_envs=15
    )
    BASE_ARGS+=(
        task.env.terrain.terrainType=plane
    )
}

a1Terrain(){
    # bash run.sh a1Terrain -p
    base
    task=A1Terrain
    PLAY_ARGS+=(
        checkpoint=assets/checkpoints/A1Terrain.pth
        # task.env.dataPublisher.enable=true
        # ++task.env.urdfAsset.AssetOptions.vhacd_enabled=true

        task.env.enableDebugVis=True
        num_envs=20
        task.env.terrain.terrainType=heightfield
        # task.env.terrain.terrainType=trimesh
        task.env.terrain.numLevels=2
        task.env.terrain.numTerrains=5
        task.env.terrain.curriculum=False
        task.env.randomCommandVelocityRanges.linear_x=[0,0]
        task.env.randomCommandVelocityRanges.linear_y=[0,0]
        task.env.randomCommandVelocityRanges.yaw=[0,0]
    )
}


anymalTerrain(){
    # bash run.sh anymalTerrain -p
    base
    task=AnymalTerrain
    PLAY_ARGS+=(
        num_envs=2
        checkpoint=assets/checkpoints/AnymalTerrain.pth
        task.env.terrain.terrainType=plane
        # task.task.randomize=true
        # env.enableDebugVis=True
    )
}

base(){
    checkpoint=null
    ENTRY_POINT=train.py
    PLAY_ARGS=(
        test=true
    )
    
    TRAIN_ARGS=(
        headless=true
    )
    BASE_ARGS=(
    )
    KEYBOARD_ARGS=(
        task.env.viewer.keyboardOperator=true
    )
}



match(){
    ENTRY_POINT=dyanmics_matching.py
    task=Biped

    BASE_ARGS+=(
        num_envs=2
        headless=false
        task.env.terrain.terrainType=plane
        ++task.env.baseHeightOffset=1
        task.env.urdfAsset.AssetOptions.fix_base_link=true
        ++task.env.defaultJointPositions=[0,1.7,0,0,0,0.000,-1.7,0,0,0]

        task.env.control.stiffness=0
        task.env.control.damping=0

        pipeline="cpu"

        ++task.env.assetDofProperties.velocity=20
        ++task.env.assetDofProperties.stiffness=0
        ++task.env.assetDofProperties.damping=0
        ++task.env.assetDofProperties.friction=0.008
        ++task.env.assetDofProperties.armature=0.2

        ++task.env.renderFPS=50
        task.env.learn.episodeLength_s=555

        task.env.urdfAsset.file=urdf/biped/v6_cylinder_leg.urdf
        task.env.urdfAsset.AssetOptions.replace_cylinder_with_capsule=true
    )
}