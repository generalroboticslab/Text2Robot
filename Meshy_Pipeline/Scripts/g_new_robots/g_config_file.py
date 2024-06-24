import os 
import yaml
import trimesh
import urdfpy
import numpy as np
import xml.etree.ElementTree as ET

def get_robot_scene(robot,cfg=None,use_collision=True):
    """Visualize the URDF in a given configuration.
    Parameters
    ----------
    cfg : dict or (n), float
        A map from joints or joint names to configuration values for
        each joint, or a list containing a value for each actuated joint
        in sorted order from the base link.
        If not specified, all joints are assumed to be in their default
        configurations.
    use_collision : bool
        If True, the collision geometry is visualized instead of
        the visual geometry.
    """
    fk = robot.collision_trimesh_fk()
    if use_collision:
        fk = robot.collision_trimesh_fk(cfg=cfg)
    else:
        fk = robot.visual_trimesh_fk(cfg=cfg)

    scene = trimesh.scene.Scene()
    for tm in fk:
        pose = fk[tm]
        mesh = tm
        scene.add_geometry(mesh, transform=pose)
    return scene

root = '../../Generated_robot_URDFS/'
for path in os.listdir(root):
    if 'urdf' in path:
        urdf_file = root+path

    robot = urdfpy.URDF.load(urdf_file)
    scene = get_robot_scene(robot)
        
    bounding_box = scene.bounding_box
    starting_position = float(-bounding_box.bounds[0,2]+0.005)
    print(starting_position)
    urdf_info = ET.parse(urdf_file).getroot()
    joint_angles = {}
    for joint in urdf_info.findall('joint'):
        joint_angles[joint.get('name')] = 0
        # joint_object_list_1.append(joint)
        # joint_list_1.append(joint.get('name'))
    # Create the YAML data structure
    data = {
        "task": {
            "env": {
                "baseInitState": {
                    "pos": [0, 0, starting_position],
                    "rot": [0, 0, 0, 1]
                },
                "baseHeightTarget": starting_position,
                "defaultJointAngles": joint_angles,
                "desiredJointAngles": joint_angles.copy()
            }
        }
    }
    yaml_filename = "../../Generated_robot_URDFS_config/"
    filename = f"{yaml_filename}{path}.yaml"
    # Write data to a YAML file
    with open(filename, "w") as file:
        yaml.dump(data, file, default_flow_style=False)
