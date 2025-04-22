# #!/bin/bash
frog_demo(){
    base
    task=RobotDog
    PLAY_ARGS+=(
        num_envs=1
        checkpoint=assets/checkpoints/Frog_Paper_Checkpoint.pth
        ++task.env.renderFPS=50
    )
    BASE_ARGS+=(
    # task.env.terrain.terrainType=plane
    ++task.env.urdfAsset.root="assets/urdf/frog_demo"
    task.env.urdfAsset.file="frog.urdf"
    task.env.randomCommandVelocityRanges.linear_x=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.linear_y=[-0.5,0.5]
    task.env.randomCommandVelocityRanges.yaw=[-1,1]

    task.env.dataPublisher.enable=true
    task.env.dataPublisher.target_url=udp://10.172.14.96:9870
    )
}

velocity_demo(){
    base
    task=RobotDog
    PLAY_ARGS+=(
        num_envs=1
        checkpoint=assets/checkpoints/Velocity_Paper_Checkpoint.pth
        ++task.env.renderFPS=50
    )
    BASE_ARGS+=(
    # task.env.terrain.terrainType=plane
    ++task.env.urdfAsset.root="assets/urdf/velocity_demo"
    task.env.urdfAsset.file="velocity.urdf"
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