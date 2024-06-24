import shutil
import xml.etree.ElementTree as ET
import os
import random
import numpy as np
from util import *
import re

# Used for genetic operators to swap and modify URDFs
class swap_legs_from_urdf(object):
    def __init__(self, robot_1, robot_2, dest_path, isMutate):
        self.dest_path = dest_path
        self.robot_1 = robot_1
        self.robot_2 = robot_2
        self.robot_1_path = dest_path + '/' + f'{robot_1}'
        self.URDF_1 = ET.parse(dest_path + '/' + f'{robot_1}/{robot_1}.urdf').getroot()

        if not isMutate:
            self.URDF_2 = ET.parse(dest_path + '/' + f'{robot_2}/{robot_2}.urdf').getroot()
        else:
            parts = dest_path.split('/', 1)
            experiment_name = parts[0] # Without /Offspring
            self.URDF_2 = ET.parse(experiment_name + '/URDF_Bank/' + f'{robot_2}/{robot_2}.urdf').getroot()

        self.id = str(random.randint(0, 10000))
        self.get_link_and_joint_info()

    def get_link_and_joint_info(self):
        self.joint_object_list_1 = []
        self.joint_list_1 = []
        self.joint_origin_list_1 = []
        self.joint_axis_list_1 = []

        for joint in self.URDF_1.findall('joint'):
            self.joint_object_list_1.append(joint)
            self.joint_list_1.append(joint.get('name'))
            joint_origin = joint.find('origin').get('xyz')
            joint_axis = joint.find('axis').get('xyz')
            self.joint_origin_list_1.append([float(i) for i in joint_origin.split(' ')])
            self.joint_axis_list_1.append([float(i) for i in joint_axis.split(' ')])

        self.joint_object_list_2 = []
        self.joint_list_2 = []
        self.joint_origin_list_2 = []
        self.joint_axis_list_2 = []

        for joint in self.URDF_2.findall('joint'):
            self.joint_object_list_2.append(joint)
            self.joint_list_2.append(joint.get('name'))
            joint_origin = joint.find('origin').get('xyz')
            joint_axis = joint.find('axis').get('xyz')
            self.joint_origin_list_2.append([float(i) for i in joint_origin.split(' ')])
            self.joint_axis_list_2.append([float(i) for i in joint_axis.split(' ')])

        self.link_list_object_1 = []

        for link in self.URDF_1.findall('link'):
            if link.get('name') == 'base_link':
                pass
            else:
                self.link_list_object_1.append(link)
                
            current_name = link.find('visual').find('geometry').find('mesh').get('filename')
            link.find('visual').find('geometry').find('mesh').set('filename', current_name)
            link.find('collision').find('geometry').find('mesh').set('filename', current_name)

        self.link_list_object_2 = []
        for link in self.URDF_2.findall('link'):
            if link.get('name') == 'base_link':
                pass
            else:
                self.link_list_object_2.append(link)
            current_name = link.find('visual').find('geometry').find('mesh').get('filename')
            link.find('visual').find('geometry').find('mesh').set('filename', current_name)
            link.find('collision').find('geometry').find('mesh').set('filename', current_name)\
            

    def swap_links(self, dest_path, level=-1):

        generated_bot_1 = None

        if(level == -1):
            random_level = random.randint(1,2)
        else:
            random_level = level

        for i in range(4):
            idx_1 = self.joint_list_1.index(f'joint_{random_level}_{i}')
            idx_2 = self.joint_list_2.index(f'joint_{random_level}_{i}')

                
            #if the selected link is upper leg, then the position of lower leg need to be adjust
            if random_level == 1:
                lower_idx_1 = self.joint_list_1.index(f'joint_2_{i}')
                lower_idx_2 = self.joint_list_2.index(f'joint_2_{i}')
                joint_1 = self.joint_object_list_1[lower_idx_1].find('origin').get('xyz')
                joint_2 = self.joint_object_list_2[lower_idx_2].find('origin').get('xyz')
                self.joint_object_list_1[lower_idx_1].find('origin').set('xyz', joint_2)
                self.joint_object_list_2[lower_idx_2].find('origin').set('xyz', joint_1)

            #swap the link position 
            link_1 = self.link_list_object_1[idx_1].find('visual').find('geometry').find('mesh').get('filename')
            link_2 = self.link_list_object_2[idx_2].find('visual').find('geometry').find('mesh').get('filename')
            link_origin_1 = self.link_list_object_1[idx_1].find('visual').find('origin').get('xyz')
            link_origin_2 = self.link_list_object_2[idx_2].find('visual').find('origin').get('xyz')

            self.link_list_object_1[idx_1].find('visual').find('geometry').find('mesh').set('filename', link_2)
            self.link_list_object_1[idx_1].find('collision').find('geometry').find('mesh').set('filename', link_2)
            self.link_list_object_1[idx_1].find('visual').find('origin').set('xyz', link_origin_2)
            self.link_list_object_1[idx_1].find('collision').find('origin').set('xyz', link_origin_2)

            self.link_list_object_2[idx_2].find('visual').find('geometry').find('mesh').set('filename', link_1)
            self.link_list_object_2[idx_2].find('collision').find('geometry').find('mesh').set('filename', link_1)
            self.link_list_object_2[idx_2].find('visual').find('origin').set('xyz', link_origin_1)
            self.link_list_object_2[idx_2].find('collision').find('origin').set('xyz', link_origin_1)
        
        # Fix naming
        robot_1_features = extract_robot_features(self.robot_1)
        robot_2_features = extract_robot_features(self.robot_2)

        # If the bottom legs are swapped:
        if random_level == 2:
            
            generated_bot_1 = assemble_robot_name(top_axis=robot_1_features['top_axis'],
                                                  bottom_axis=robot_1_features['bottom_axis'],
                                                  top_name=robot_1_features['top_name'],
                                                  bottom_name=robot_2_features['bottom_name'],
                                                  top_scale=robot_1_features['top_scale'],
                                                  bottom_scale=robot_2_features['bottom_scale'],
                                                  baselink_name=robot_1_features['base_link_name'])
        
        if random_level == 1:
            
            generated_bot_1 = assemble_robot_name(top_axis=robot_1_features['top_axis'],
                                                  bottom_axis=robot_1_features['bottom_axis'],
                                                  top_name=robot_2_features['top_name'],
                                                  bottom_name=robot_1_features['bottom_name'],
                                                  top_scale=robot_2_features['top_scale'],
                                                  bottom_scale=robot_1_features['bottom_scale'],
                                                  baselink_name=robot_1_features['base_link_name'])

        directory1 = Path(f"{dest_path}/{generated_bot_1}")

        if directory1.exists():
            return None

        create_directory(directory1)

        tree_1 = ET.ElementTree(self.URDF_1)
        tree_1.write(f"{dest_path}/{generated_bot_1}/{generated_bot_1}.urdf", encoding='utf-8', xml_declaration=True)
        
        return generated_bot_1

    def swap_joints(self, dest_path, level=-1):
        if(level == -1):
            random_level = random.randint(1,2)
        else:
            random_level = level

        generated_bot_1 = None
        
        for i in range(4):
            idx_1 = self.joint_list_1.index(f'joint_{random_level}_{i}')
            idx_2 = self.joint_list_2.index(f'joint_{random_level}_{i}')
            
            # Don't attempt to swap the joints if they are already the same
            if self.joint_object_list_2[idx_1].find('axis').get('xyz') == self.joint_object_list_1[idx_2].find('axis').get('xyz'):
                return None
            else:
                self.joint_object_list_1[idx_1].find('axis').set('xyz', f'{self.joint_axis_list_2[idx_1][0]} {self.joint_axis_list_2[idx_1][1]} {self.joint_axis_list_2[idx_1][2]}')
                self.joint_object_list_2[idx_2].find('axis').set('xyz', f'{self.joint_axis_list_1[idx_2][0]} {self.joint_axis_list_1[idx_2][1]} {self.joint_axis_list_1[idx_2][2]}')

        # Fix naming
        robot_1_features = extract_robot_features(self.robot_1)
        robot_2_features = extract_robot_features(self.robot_2)

        # If the bottom legs are swapped:
        if random_level == 2:

            generated_bot_1 = assemble_robot_name(top_axis=robot_1_features['top_axis'],
                                                  bottom_axis=robot_2_features['bottom_axis'],
                                                  top_name=robot_1_features['top_name'],
                                                  bottom_name=robot_1_features['bottom_name'],
                                                  top_scale=robot_1_features['top_scale'],
                                                  bottom_scale=robot_1_features['bottom_scale'],
                                                  baselink_name=robot_1_features['base_link_name'])

        if random_level == 1:

            generated_bot_1 = assemble_robot_name(top_axis=robot_2_features['top_axis'],
                                                  bottom_axis=robot_1_features['bottom_axis'],
                                                  top_name=robot_1_features['top_name'],
                                                  bottom_name=robot_1_features['bottom_name'],
                                                  top_scale=robot_1_features['top_scale'],
                                                  bottom_scale=robot_1_features['bottom_scale'],
                                                  baselink_name=robot_1_features['base_link_name'])

        directory1 = Path(f"{dest_path}/{generated_bot_1}")

        if directory1.exists():
            return None

        create_directory(directory1)

        tree_1 = ET.ElementTree(self.URDF_1)
        tree_1.write(f"{dest_path}/{generated_bot_1}/{generated_bot_1}.urdf", encoding='utf-8', xml_declaration=True)
        
        return generated_bot_1

    def mutate_links(self, dest_path, level):
        random_level = level

        for i in range(4):
            idx_1 = self.joint_list_1.index(f'joint_{random_level}_{i}')
            idx_2 = self.joint_list_2.index(f'joint_{random_level}_{i}')

                
            #if the selected link is upper leg, then the position of lower leg need to be adjust
            if random_level == 1:
                lower_idx_1 = self.joint_list_1.index(f'joint_2_{i}')
                lower_idx_2 = self.joint_list_2.index(f'joint_2_{i}')
                joint_1 = self.joint_object_list_1[lower_idx_1].find('origin').get('xyz')
                joint_2 = self.joint_object_list_2[lower_idx_2].find('origin').get('xyz')
                self.joint_object_list_1[lower_idx_1].find('origin').set('xyz', joint_2)
                self.joint_object_list_2[lower_idx_2].find('origin').set('xyz', joint_1)

            #swap the link position 
            link_1 = self.link_list_object_1[idx_1].find('visual').find('geometry').find('mesh').get('filename')
            link_2 = self.link_list_object_2[idx_2].find('visual').find('geometry').find('mesh').get('filename')
            link_origin_1 = self.link_list_object_1[idx_1].find('visual').find('origin').get('xyz')
            link_origin_2 = self.link_list_object_2[idx_2].find('visual').find('origin').get('xyz')

            self.link_list_object_1[idx_1].find('visual').find('geometry').find('mesh').set('filename', link_2)
            self.link_list_object_1[idx_1].find('collision').find('geometry').find('mesh').set('filename', link_2)
            self.link_list_object_1[idx_1].find('visual').find('origin').set('xyz', link_origin_2)
            self.link_list_object_1[idx_1].find('collision').find('origin').set('xyz', link_origin_2)

            self.link_list_object_2[idx_2].find('visual').find('geometry').find('mesh').set('filename', link_1)
            self.link_list_object_2[idx_2].find('collision').find('geometry').find('mesh').set('filename', link_1)
            self.link_list_object_2[idx_2].find('visual').find('origin').set('xyz', link_origin_1)
            self.link_list_object_2[idx_2].find('collision').find('origin').set('xyz', link_origin_1)
        
        # Fix naming
        robot_1_features = extract_robot_features(self.robot_1)
        robot_2_features = extract_robot_features(self.robot_2)

        generated_robot = None

        # If the bottom legs are swapped:
        if random_level == 2:
            generated_robot = assemble_robot_name(top_axis=robot_1_features['top_axis'],
                                                  bottom_axis=robot_1_features['bottom_axis'],
                                                  top_name=robot_1_features['top_name'],
                                                  bottom_name=robot_2_features['bottom_name'],
                                                  top_scale=robot_1_features['top_scale'],
                                                  bottom_scale=robot_2_features['bottom_scale'],
                                                  baselink_name=robot_1_features['base_link_name'])
            
        if random_level == 1:
            generated_robot = assemble_robot_name(top_axis=robot_1_features['top_axis'],
                                                  bottom_axis=robot_1_features['bottom_axis'],
                                                  top_name=robot_2_features['top_name'],
                                                  bottom_name=robot_1_features['bottom_name'],
                                                  top_scale=robot_2_features['top_scale'],
                                                  bottom_scale=robot_1_features['bottom_scale'],
                                                  baselink_name=robot_1_features['base_link_name'])
            
        directory1 = Path(f"{dest_path}/{generated_robot}")

        if directory1.exists():
            return None
        
        create_directory(directory1)

        tree_1 = ET.ElementTree(self.URDF_1)
        new_urdf_path = f"{dest_path}/{generated_robot}/{generated_robot}.urdf"
        tree_1.write(new_urdf_path, encoding='utf-8', xml_declaration=True)
        modify_urdf_file(new_urdf_path, self.robot_2) # Fix relative path to meshes in swap from bank
        
        return generated_robot
    
    def mutate_baselink(self, dest_path):

        level = 1
        for i in range(4):
            idx_1 = self.joint_list_1.index(f'joint_{level}_{i}')
            idx_2 = self.joint_list_2.index(f'joint_{level}_{i}')
                
            #the selected link is upper leg, so the position of lower leg need to be adjust
            lower_idx_1 = self.joint_list_1.index(f'joint_2_{i}')
            lower_idx_2 = self.joint_list_2.index(f'joint_2_{i}')
            joint_1 = self.joint_object_list_1[lower_idx_1].find('origin').get('xyz')
            joint_2 = self.joint_object_list_2[lower_idx_2].find('origin').get('xyz')
            self.joint_object_list_1[lower_idx_1].find('origin').set('xyz', joint_2)
            self.joint_object_list_2[lower_idx_2].find('origin').set('xyz', joint_1)

            #swap the link position 
            link_1 = self.link_list_object_1[idx_1].find('visual').find('geometry').find('mesh').get('filename')
            link_2 = self.link_list_object_2[idx_2].find('visual').find('geometry').find('mesh').get('filename')
            link_origin_1 = self.link_list_object_1[idx_1].find('visual').find('origin').get('xyz')
            link_origin_2 = self.link_list_object_2[idx_2].find('visual').find('origin').get('xyz')

            self.link_list_object_1[idx_1].find('visual').find('geometry').find('mesh').set('filename', link_2)
            self.link_list_object_1[idx_1].find('collision').find('geometry').find('mesh').set('filename', link_2)
            self.link_list_object_1[idx_1].find('visual').find('origin').set('xyz', link_origin_2)
            self.link_list_object_1[idx_1].find('collision').find('origin').set('xyz', link_origin_2)

            self.link_list_object_2[idx_2].find('visual').find('geometry').find('mesh').set('filename', link_1)
            self.link_list_object_2[idx_2].find('collision').find('geometry').find('mesh').set('filename', link_1)
            self.link_list_object_2[idx_2].find('visual').find('origin').set('xyz', link_origin_1)
            self.link_list_object_2[idx_2].find('collision').find('origin').set('xyz', link_origin_1)

        for i in range(4):
            idx_1 = self.joint_list_1.index(f'joint_{level}_{i}')
            idx_2 = self.joint_list_2.index(f'joint_{level}_{i}')
            
            # Don't attempt to swap the joints if they are already the same
            if self.joint_object_list_2[idx_1].find('axis').get('xyz') == self.joint_object_list_1[idx_2].find('axis').get('xyz'):
                continue
            else:
                self.joint_object_list_1[idx_1].find('axis').set('xyz', f'{self.joint_axis_list_2[idx_1][0]} {self.joint_axis_list_2[idx_1][1]} {self.joint_axis_list_2[idx_1][2]}')
                self.joint_object_list_2[idx_2].find('axis').set('xyz', f'{self.joint_axis_list_1[idx_2][0]} {self.joint_axis_list_1[idx_2][1]} {self.joint_axis_list_1[idx_2][2]}')

        level = 2
        for i in range(4):
            idx_1 = self.joint_list_1.index(f'joint_{level}_{i}')
            idx_2 = self.joint_list_2.index(f'joint_{level}_{i}')

            #swap the link position 
            link_1 = self.link_list_object_1[idx_1].find('visual').find('geometry').find('mesh').get('filename')
            link_2 = self.link_list_object_2[idx_2].find('visual').find('geometry').find('mesh').get('filename')
            link_origin_1 = self.link_list_object_1[idx_1].find('visual').find('origin').get('xyz')
            link_origin_2 = self.link_list_object_2[idx_2].find('visual').find('origin').get('xyz')

            self.link_list_object_1[idx_1].find('visual').find('geometry').find('mesh').set('filename', link_2)
            self.link_list_object_1[idx_1].find('collision').find('geometry').find('mesh').set('filename', link_2)
            self.link_list_object_1[idx_1].find('visual').find('origin').set('xyz', link_origin_2)
            self.link_list_object_1[idx_1].find('collision').find('origin').set('xyz', link_origin_2)

            self.link_list_object_2[idx_2].find('visual').find('geometry').find('mesh').set('filename', link_1)
            self.link_list_object_2[idx_2].find('collision').find('geometry').find('mesh').set('filename', link_1)
            self.link_list_object_2[idx_2].find('visual').find('origin').set('xyz', link_origin_1)
            self.link_list_object_2[idx_2].find('collision').find('origin').set('xyz', link_origin_1)

        for i in range(4):
            idx_1 = self.joint_list_1.index(f'joint_{level}_{i}')
            idx_2 = self.joint_list_2.index(f'joint_{level}_{i}')
            
            # Don't attempt to swap the joints if they are already the same
            if self.joint_object_list_2[idx_1].find('axis').get('xyz') == self.joint_object_list_1[idx_2].find('axis').get('xyz'):
                continue
            else:
                self.joint_object_list_1[idx_1].find('axis').set('xyz', f'{self.joint_axis_list_2[idx_1][0]} {self.joint_axis_list_2[idx_1][1]} {self.joint_axis_list_2[idx_1][2]}')
                self.joint_object_list_2[idx_2].find('axis').set('xyz', f'{self.joint_axis_list_1[idx_2][0]} {self.joint_axis_list_1[idx_2][1]} {self.joint_axis_list_1[idx_2][2]}')

        # Fix naming
        robot_1_features = extract_robot_features(self.robot_1)
        robot_2_features = extract_robot_features(self.robot_2)

        generated_robot = None

        generated_robot = assemble_robot_name(top_axis=robot_1_features['top_axis'],
                                                bottom_axis=robot_1_features['bottom_axis'],
                                                top_name=robot_1_features['top_name'],
                                                bottom_name=robot_1_features['bottom_name'],
                                                top_scale=robot_1_features['top_scale'],
                                                bottom_scale=robot_1_features['bottom_scale'],
                                                baselink_name=robot_2_features['base_link_name'])
        
            
        directory1 = Path(f"{dest_path}/{generated_robot}")

        if directory1.exists():
            return None
        
        create_directory(directory1)

        tree_1 = ET.ElementTree(self.URDF_2)
        new_urdf_path = f"{dest_path}/{generated_robot}/{generated_robot}.urdf"
        tree_1.write(new_urdf_path, encoding='utf-8', xml_declaration=True)
        modify_urdf_file(new_urdf_path, self.robot_2) # Fix relative path to meshes in swap from bank
        
        return generated_robot

    def mutate_joints(self, dest_path, level):
        random_level = level
        
        for i in range(4):
            idx_1 = self.joint_list_1.index(f'joint_{random_level}_{i}')
            idx_2 = self.joint_list_2.index(f'joint_{random_level}_{i}')
            
            # Don't attempt to swap the joints if they are already the same
            if self.joint_object_list_2[idx_1].find('axis').get('xyz') == self.joint_object_list_1[idx_2].find('axis').get('xyz'):
                return
            else:
                self.joint_object_list_1[idx_1].find('axis').set('xyz', f'{self.joint_axis_list_2[idx_1][0]} {self.joint_axis_list_2[idx_1][1]} {self.joint_axis_list_2[idx_1][2]}')
                self.joint_object_list_2[idx_2].find('axis').set('xyz', f'{self.joint_axis_list_1[idx_2][0]} {self.joint_axis_list_1[idx_2][1]} {self.joint_axis_list_1[idx_2][2]}')

        # Fix naming
        robot_1_features = extract_robot_features(self.robot_1)
        robot_2_features = extract_robot_features(self.robot_2)

        generated_robot = None

        # If the bottom legs are swapped:
        if random_level == 2:
            generated_robot = assemble_robot_name(top_axis=robot_1_features['top_axis'],
                                                  bottom_axis=robot_2_features['bottom_axis'],
                                                  top_name=robot_1_features['top_name'],
                                                  bottom_name=robot_1_features['bottom_name'],
                                                  top_scale=robot_1_features['top_scale'],
                                                  bottom_scale=robot_1_features['bottom_scale'],
                                                  baselink_name=robot_1_features['base_link_name'])

        if random_level == 1:
            generated_robot = assemble_robot_name(top_axis=robot_2_features['top_axis'],
                                                  bottom_axis=robot_1_features['bottom_axis'],
                                                  top_name=robot_1_features['top_name'],
                                                  bottom_name=robot_1_features['bottom_name'],
                                                  top_scale=robot_1_features['top_scale'],
                                                  bottom_scale=robot_1_features['bottom_scale'],
                                                  baselink_name=robot_1_features['base_link_name'])
    
        directory1 = Path(f"{dest_path}/{generated_robot}")

        if directory1.exists():
            return None
        
        create_directory(directory1)
        tree_1 = ET.ElementTree(self.URDF_1)
        tree_1.write(f"{dest_path}/{generated_robot}/{generated_robot}.urdf", encoding='utf-8', xml_declaration=True)
        
        return generated_robot

