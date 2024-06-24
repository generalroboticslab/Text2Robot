"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.

Joint Monkey
------------
- Animates degree-of-freedom ranges for a given asset.
- Demonstrates usage of DOF properties and states.
- Demonstrates line drawing utilities to visualize DOF frames (origin and axis).
"""

import os
os.environ['LD_LIBRARY_PATH'] = f"{os.environ['CONDA_PREFIX']}/lib"
print(os.environ['LD_LIBRARY_PATH'])

import math
import numpy as np
from isaacgym import gymapi, gymutil

def clamp(x, min_value, max_value):
    return max(min(x, max_value), min_value)

# simple asset descriptor for selecting from a list


class AssetDesc:
    def __init__(self, file_name, flip_visual_attachments=False):
        self.file_name = file_name
        self.flip_visual_attachments = flip_visual_attachments


asset_descriptors = [
    # AssetDesc("urdf/bipedv6/bipedv6.urdf", False),
    AssetDesc("urdf/RobotDog/RobotDog3kg.urdf", False),
    # AssetDesc("urdf/RobotDog/RobotDog7kg.urdf", False),
    # AssetDesc("urdf/a1/a1_minimum.urdf", False),
    # AssetDesc("urdf/a1/a1_minimum_anymal_like.urdf", False),
    # AssetDesc("urdf/a1/a1_simple.urdf", False),
    # AssetDesc("urdf/a1/a1_ros.urdf", True),
    # AssetDesc("urdf/a1/a1_pybullet.urdf", False),
    # AssetDesc("urdf/anymal_c/urdf/anymal.urdf", True),
    # AssetDesc("urdf/anymal_c/urdf/anymal_minimal.urdf", True),
    # AssetDesc("urdf/anymal_c/urdf/anymal_minimal_a1_like.urdf", True),
    # AssetDesc("urdf/px2_et1/urdf/px2_et1.urdf", False),
    # AssetDesc("urdf/px2_et1/urdf/px2_et1_minimal.urdf", False),

]


# parse arguments
args = gymutil.parse_arguments(
    description="Joint monkey: Animate degree-of-freedom ranges",
    custom_parameters=[
        {"name": "--asset_id", "type": int, "default": 0, "help": "Asset id (0 - %d)" % (len(asset_descriptors) - 1)},
        {"name": "--speed_scale", "type": float, "default": 1.0, "help": "Animation speed scale"},
        {"name": "--show_axis", "action": "store_true", "help": "Visualize DOF axis"}])

if args.asset_id < 0 or args.asset_id >= len(asset_descriptors):
    print("*** Invalid asset_id specified.  Valid range is 0 to %d" % (len(asset_descriptors) - 1))
    quit()

# args.show_axis = True

for k in range(50):
    # initialize gym
    gym = gymapi.acquire_gym()

    # configure sim
    sim_params = gymapi.SimParams()
    sim_params.dt = dt = 1.0 / 60.0
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0,0,0)
    if args.physics_engine == gymapi.SIM_FLEX:
        pass
    elif args.physics_engine == gymapi.SIM_PHYSX:
        sim_params.physx.solver_type = 1
        sim_params.physx.num_position_iterations = 4
        sim_params.physx.num_velocity_iterations = 0
        sim_params.physx.num_threads = args.num_threads
        sim_params.physx.use_gpu = args.use_gpu


    sim_params.use_gpu_pipeline = False
    if args.use_gpu_pipeline:
        print("WARNING: Forcing CPU pipeline.")

    sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
    if sim is None:
        print("*** Failed to create sim")
        quit()

    # # add ground plane
    # plane_params = gymapi.PlaneParams()
    # plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
    # gym.add_ground(sim, plane_params)

    # create viewer
    viewer = gym.create_viewer(sim, gymapi.CameraProperties())
    if viewer is None:
        print("*** Failed to create viewer")
        quit()

    # set up the env grid
    num_envs = 1
    num_per_row = 2
    spacing = 1.
    env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
    env_upper = gymapi.Vec3(spacing, spacing, spacing)

    # position the camera
    # cam_pos = gymapi.Vec3(17.2, 2.0, 16)
    # cam_target = gymapi.Vec3(5, -2.5, 13)
    cam_pos = gymapi.Vec3(0, -0.5, 1.3)
    # cam_pos = gymapi.Vec3(0.4, -0.8, 1.7)
    cam_target = gymapi.Vec3(0, 0, 1.15)
    gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

    # cache useful handles
    envs = []
    actor_handles = []

    # joint animation states
    ANIM_SEEK_LOWER = 1
    ANIM_SEEK_UPPER = 2
    ANIM_SEEK_DEFAULT = 3
    ANIM_FINISHED = 4

    #------------------------------------------------------------
    # load asset
    asset_root = os.path.dirname(__file__)
    asset_file = asset_descriptors[args.asset_id].file_name

    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = True
    asset_options.flip_visual_attachments = asset_descriptors[args.asset_id].flip_visual_attachments
    asset_options.use_mesh_materials = True
    asset_options.collapse_fixed_joints = True
    asset_options.replace_cylinder_with_capsule = True
    asset_options.override_inertia = False

    asset_options.density = 0.001
    asset_options.angular_damping = 0.0
    asset_options.linear_damping = 0.0
    asset_options.armature = 0.0
    asset_options.thickness = 0.01
    asset_options.disable_gravity = False

    print("Loading asset '%s' from '%s'" % (asset_file, asset_root))
    asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

    # get array of DOF names
    dof_names = gym.get_asset_dof_names(asset)

    # get array of DOF properties
    dof_props = gym.get_asset_dof_properties(asset)

    # Maps rigid body names to asset-relative indices
    rigid_body_dict = gym.get_asset_rigid_body_dict(asset)

    # Maps joint names to asset-relative indices
    joint_dict = gym.get_asset_joint_dict(asset)

    # create an array of DOF states that will be used to update the actors
    num_dofs = gym.get_asset_dof_count(asset)
    dof_states = np.zeros(num_dofs, dtype=gymapi.DofState.dtype)

    # get list of DOF types
    dof_types = [gym.get_asset_dof_type(asset, i) for i in range(num_dofs)]

    # get the position slice of the DOF state array
    dof_positions = dof_states['pos']

    # get the limit-related slices of the DOF properties array
    stiffnesses = dof_props['stiffness']
    dampings = dof_props['damping']
    armatures = dof_props['armature']
    has_limits = dof_props['hasLimits']
    lower_limits = dof_props['lower']
    upper_limits = dof_props['upper']

    # limits = np.column_stack((lower_limits,upper_limits))
    # print("limits=",repr(limits))
    # print("lower_limits=",repr(lower_limits))
    # print("upper_limits=",repr(upper_limits))
    # exit()

    # Print DOF properties
    for i in range(num_dofs):
        print("DOF %d" % i)
        print("  Name:     '%s'" % dof_names[i])
        print("  Type:     %s" % gym.get_dof_type_string(dof_types[i]))
        print("  Stiffness:  %r" % stiffnesses[i])
        print("  Damping:  %r" % dampings[i])
        print("  Armature:  %r" % armatures[i])
        print("  Limited?  %r" % has_limits[i])
        if has_limits[i]:
            print("    Lower   %f" % lower_limits[i])
            print("    Upper   %f" % upper_limits[i])

    # initialize default positions, limits, and speeds (make sure they are in reasonable ranges)
    defaults = np.zeros(num_dofs)
    speeds = np.zeros(num_dofs)
    for i in range(num_dofs):
        if has_limits[i]:
            if dof_types[i] == gymapi.DOF_ROTATION:
                lower_limits[i] = clamp(lower_limits[i], -math.pi, math.pi)
                upper_limits[i] = clamp(upper_limits[i], -math.pi, math.pi)
            # make sure our default position is in range
            if lower_limits[i] > 0.0:
                defaults[i] = lower_limits[i]
            elif upper_limits[i] < 0.0:
                defaults[i] = upper_limits[i]
        else:
            # set reasonable animation limits for unlimited joints
            if dof_types[i] == gymapi.DOF_ROTATION:
                # unlimited revolute joint
                lower_limits[i] = -math.pi
                upper_limits[i] = math.pi
            elif dof_types[i] == gymapi.DOF_TRANSLATION:
                # unlimited prismatic joint
                lower_limits[i] = -1.0
                upper_limits[i] = 1.0
        # set DOF position to default
        dof_positions[i] = defaults[i]
        # set speed depending on DOF type and range of motion
        if dof_types[i] == gymapi.DOF_ROTATION:
            speeds[i] = args.speed_scale * clamp(2 * (upper_limits[i] - lower_limits[i]), 0.25 * math.pi, 3.0 * math.pi)
        else:
            speeds[i] = args.speed_scale * clamp(2 * (upper_limits[i] - lower_limits[i]), 0.1, 7.0)

    print("Creating %d environments" % num_envs)
    for i in range(num_envs):
        # create env
        env = gym.create_env(sim, env_lower, env_upper, num_per_row)
        envs.append(env)

        # add actor
        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0.0, 0.0, 1.32)
        pose.r = gymapi.Quat(0, 0.0, 0.0, 1) #xyzw

        actor_handle = gym.create_actor(env, asset, pose, "actor", i, 1)
        actor_handles.append(actor_handle)

        # set default DOF positions
        gym.set_actor_dof_states(env, actor_handle, dof_states, gymapi.STATE_ALL)

    #-------------------
    rigid_body_names =  gym.get_actor_rigid_body_names(env,actor_handle)
    rigid_body_properties = gym.get_actor_rigid_body_properties(env,actor_handle)
    rigid_body_masses = [p.mass for p in rigid_body_properties]
    rigid_body_inertias = [p.inertia for p in rigid_body_properties]

    total_mass = np.sum(rigid_body_masses)
    for name,mass in zip(rigid_body_names,rigid_body_masses):
        print(f"{name:20s},{mass:.5f}")
    print(f"total mass: {total_mass} kg")

    for name,i in zip(rigid_body_names,rigid_body_inertias):
        print(f"{name:20s},{i.x.x:<+8.4e},{i.x.y:<+8.4e},{i.x.z:<+8.4e},{i.y.y:<+8.4e},{i.y.z:<+8.4e},{i.z.z:<+8.4e}")
    # #-------------------
    # gym.destroy_viewer(viewer)
    # gym.destroy_sim(sim)
    # exit()
    # #-------------------

    # initialize animation state
    anim_state = ANIM_SEEK_LOWER
    current_dof = 0
    print("Animating DOF %d ('%s')" % (current_dof, dof_names[current_dof]))
    # -----------------------------------------------------------------------------

    # subscribe to keyboard events
    gym.subscribe_viewer_keyboard_event(viewer, gymapi.KEY_F, "freeze")
    gym.subscribe_viewer_keyboard_event(viewer, gymapi.KEY_R, "reset")

    init = False
    freeze = True
    while not gym.query_viewer_has_closed(viewer):
        # Get input actions from the viewer and handle them appropriately
        for evt in gym.query_viewer_action_events(viewer):
            if evt.action == "reset" and evt.value > 0:
                print("reset pressed")
                init = True
                # gym.set_sim_rigid_body_states(sim, initial_state, gymapi.STATE_ALL)
            elif evt.action=="freeze":
                freeze = not freeze
        if init:
            break
        # step the physics
        gym.simulate(sim)
        gym.fetch_results(sim, True)

        speed = speeds[current_dof]

        # animate the dofs
        if anim_state == ANIM_SEEK_LOWER:
            dof_positions[current_dof] -= speed * dt
            if dof_positions[current_dof] <= lower_limits[current_dof]:
                dof_positions[current_dof] = lower_limits[current_dof]
                anim_state = ANIM_SEEK_UPPER
        elif anim_state == ANIM_SEEK_UPPER:
            dof_positions[current_dof] += speed * dt
            if dof_positions[current_dof] >= upper_limits[current_dof]:
                dof_positions[current_dof] = upper_limits[current_dof]
                anim_state = ANIM_SEEK_DEFAULT
        if anim_state == ANIM_SEEK_DEFAULT:
            dof_positions[current_dof] -= speed * dt
            if dof_positions[current_dof] <= defaults[current_dof]:
                dof_positions[current_dof] = defaults[current_dof]
                anim_state = ANIM_FINISHED
        elif anim_state == ANIM_FINISHED:
            dof_positions[current_dof] = defaults[current_dof]
            current_dof = (current_dof + 1) % num_dofs
            anim_state = ANIM_SEEK_LOWER
            print("Animating DOF %d ('%s')" % (current_dof, dof_names[current_dof]))

        # if args.show_axis:
        gym.clear_lines(viewer)

        # clone actor state in all of the environments
        for i in range(num_envs):
            if not freeze:
                gym.set_actor_dof_states(envs[i], actor_handles[i], dof_states, gymapi.STATE_POS)

            if args.show_axis:
                # get the DOF frame (origin and axis)
                dof_handle = gym.get_actor_dof_handle(envs[i], actor_handles[i], current_dof)
                frame = gym.get_dof_frame(envs[i], dof_handle)

                # draw a line from DOF origin along the DOF axis
                p1 = frame.origin
                p2 = frame.origin + frame.axis * 0.7
                color = gymapi.Vec3(1.0, 0.0, 0.0)
                gymutil.draw_line(p1, p2, color, gym, viewer, envs[i])

            #show all axis:
            for k in range(num_dofs):
                # get the DOF frame (origin and axis)
                dof_handle = gym.get_actor_dof_handle(envs[i], actor_handles[i], k)
                frame = gym.get_dof_frame(envs[i], dof_handle)

                # draw a line from DOF origin along the DOF axis
                p1 = frame.origin
                p2 = frame.origin + frame.axis * 0.7
                color = gymapi.Vec3(1.0, 0.0, 0.0)
                gymutil.draw_line(p1, p2, color, gym, viewer, envs[i])

        # update the viewer
        gym.step_graphics(sim)
        gym.draw_viewer(viewer, sim, False)

        # Wait for dt to elapse in real time.
        # This synchronizes the physics simulation with the rendering rate.
        gym.sync_frame_time(sim)

    print("Done")

    gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)
