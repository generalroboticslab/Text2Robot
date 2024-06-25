# #!/bin/bash
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