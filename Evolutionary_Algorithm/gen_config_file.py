import os 
import yaml
import trimesh
import urdfpy
import numpy as np
import xml.etree.ElementTree as ET

class g_config_file(object):
    def __init__(self,root):
        self.g_config(root)

    def g_config(self,root):
        for robot_name in os.listdir(root):
            if robot_name == 'log_files': # Don't attempt to evaluate log_files
                continue
            needs_evaluted = True
            for file_name in os.listdir(root+'/'+robot_name):
                if 'yaml' in file_name:
                    needs_evaluted = False
                if 'urdf' in file_name:
                    urdf_file = root+'/'+robot_name+'/'+file_name

            if needs_evaluted == False: # Don't re-evaluate bots that already have yaml files
                continue

            robot = urdfpy.URDF.load(urdf_file)
            scene = self. get_robot_scene(robot)
            bounding_box = scene.bounding_box
            # bounding_box.show()

            starting_position = float(-bounding_box.bounds[0,2]+0.005)
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
            filename = f"{urdf_file.replace('urdf','yaml')}"
            # Write data to a YAML file
            with open(filename, "w") as file:
                yaml.dump(data, file, default_flow_style=False)


    def get_robot_scene(self, robot,cfg=None,use_collision=False):
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
        # scene.show()
        return scene
        
    
