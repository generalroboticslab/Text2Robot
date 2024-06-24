import shutil
import xml.etree.ElementTree as ET
import os
import random
import numpy as np

class swap_legs_from_urdf(object):
    def __init__(self,robot_1, robot_2):
        self.robot_1 = robot_1
        self.robot_2 = robot_2
        self.URDF_1 = ET.parse(root_path+f'{robot_1}/{robot_1}.urdf').getroot()
        self.URDF_2 = ET.parse(root_path+f'{robot_2}/{robot_2}.urdf').getroot()
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
            link.find('visual').find('geometry').find('mesh').set('filename', '../URDFsForBoxi/'+self.robot_1+'/'+current_name)
            link.find('collision').find('geometry').find('mesh').set('filename', '../URDFsForBoxi/'+self.robot_1+'/'+current_name)

        self.link_list_object_2 = []
        for link in self.URDF_2.findall('link'):
            if link.get('name') == 'base_link':
                pass
            else:
                self.link_list_object_2.append(link)
            current_name = link.find('visual').find('geometry').find('mesh').get('filename')
            link.find('visual').find('geometry').find('mesh').set('filename', '../URDFsForBoxi/'+self.robot_2+'/'+current_name)
            link.find('collision').find('geometry').find('mesh').set('filename', '../URDFsForBoxi/'+self.robot_2+'/'+current_name)\
            

    def swap_links(self):
        while True:
            #randomly select the same link for two robot
            random_choice= random.randint(0,len(self.joint_list_1)-1)
            random_choice_1_index = self.joint_list_1[random_choice].split('_') 
            random_choice_2_index = self.joint_list_2[random_choice].split('_') 
            if random_choice_2_index[1] == random_choice_1_index[1]:
                idx_1 = self.joint_list_1.index(f'{random_choice_1_index[0]}_2_{random_choice_1_index[-1]}')
                idx_2 = self.joint_list_2.index(f'{random_choice_2_index[0]}_2_{random_choice_2_index[-1]}')
                break 
            
        #if the selected link is upper leg, then the position of lower leg need to be adjust
        if random_choice_2_index[1] == '1':
            joint_1 = self.joint_object_list_1[idx_1].find('origin').get('xyz')
            joint_2 = self.joint_object_list_2[idx_2].find('origin').get('xyz')
            self.joint_object_list_1[idx_1].find('origin').set('xyz', joint_2)
            self.joint_object_list_2[idx_2].find('origin').set('xyz', joint_1)

        #swap the link position 
        link_1 = self.link_list_object_1[random_choice].find('visual').find('geometry').find('mesh').get('filename')
        link_2 = self.link_list_object_2[random_choice].find('visual').find('geometry').find('mesh').get('filename')
        link_origin_1 = self.link_list_object_1[random_choice].find('visual').find('origin').get('xyz')
        link_origin_2 = self.link_list_object_2[random_choice].find('visual').find('origin').get('xyz')

        self.link_list_object_1[random_choice].find('visual').find('geometry').find('mesh').set('filename', link_2)
        self.link_list_object_1[random_choice].find('collision').find('geometry').find('mesh').set('filename', link_2)
        self.link_list_object_1[random_choice].find('visual').find('origin').set('xyz', link_origin_2)
        self.link_list_object_1[random_choice].find('collision').find('origin').set('xyz', link_origin_2)

        self.link_list_object_2[random_choice].find('visual').find('geometry').find('mesh').set('filename', link_1)
        self.link_list_object_2[random_choice].find('collision').find('geometry').find('mesh').set('filename', link_1)
        self.link_list_object_2[random_choice].find('visual').find('origin').set('xyz', link_origin_1)
        self.link_list_object_2[random_choice].find('collision').find('origin').set('xyz', link_origin_1)

        id = random.randint(0, 10000)
        print(id)
        print(self.robot_1,self.robot_2)
        tree_1 = ET.ElementTree(self.URDF_1)
        tree_1.write(f'../../Generated_robot_URDFS/{self.robot_1}_link_ID-{id}.urdf', encoding='utf-8', xml_declaration=True)
        tree_2 = ET.ElementTree(self.URDF_2)
        tree_2.write(f'../../Generated_robot_URDFS/{self.robot_2}_link_ID-{id}.urdf', encoding='utf-8', xml_declaration=True)


    def swap_joints(self):
        while True:
                #randomly select the same link for two robot
            random_choice= random.randint(0,len(self.joint_list_1)-1)
            random_choice_1_index = self.joint_list_1[random_choice].split('_') 
            random_choice_2_index = self.joint_list_2[random_choice].split('_') 
                
            # print(self.joint_object_list_2[random_choice].find('axis').get('xyz'))
            # print(self.joint_object_list_1[random_choice].find('axis').get('xyz'))
            if self.joint_object_list_2[random_choice].find('axis').get('xyz') != self.joint_object_list_1[random_choice].find('axis').get('xyz'):
            
            
                self.joint_object_list_1[random_choice].find('axis').set('xyz', f'{self.joint_axis_list_2[random_choice][0]} {self.joint_axis_list_2[random_choice][1]} {self.joint_axis_list_2[random_choice][2]}')
                self.joint_object_list_2[random_choice].find('axis').set('xyz', f'{self.joint_axis_list_1[random_choice][0]} {self.joint_axis_list_1[random_choice][1]} {self.joint_axis_list_1[random_choice][2]}')

            # link_1 = self.link_list_object_1[random_choice].find('visual').find('geometry').find('mesh').get('filename')
            # link_2 = self.link_list_object_2[random_choice].find('visual').find('geometry').find('mesh').get('filename')
            # link_origin_1 = self.link_list_object_1[random_choice].find('visual').find('origin').get('xyz')
            # link_origin_2 = self.link_list_object_2[random_choice].find('visual').find('origin').get('xyz')

            # self.link_list_object_1[random_choice].find('visual').find('geometry').find('mesh').set('filename', link_2)
            # self.link_list_object_1[random_choice].find('collision').find('geometry').find('mesh').set('filename', link_2)
            # self.link_list_object_1[random_choice].find('visual').find('origin').set('xyz', link_origin_2)
            # self.link_list_object_1[random_choice].find('collision').find('origin').set('xyz', link_origin_2)

            # self.link_list_object_2[random_choice].find('visual').find('geometry').find('mesh').set('filename', link_1)
            # self.link_list_object_2[random_choice].find('collision').find('geometry').find('mesh').set('filename', link_1)
            # self.link_list_object_2[random_choice].find('visual').find('origin').set('xyz', link_origin_1)
            # self.link_list_object_2[random_choice].find('collision').find('origin').set('xyz', link_origin_1)

                id = random.randint(0, 10000)
                print(id)
                tree_1 = ET.ElementTree(self.URDF_1)
                tree_1.write(f'../../Generated_robot_URDFS/{self.robot_1}_joint_ID-{id}.urdf', encoding='utf-8', xml_declaration=True)
                tree_2 = ET.ElementTree(self.URDF_2)
                tree_2.write(f'../../Generated_robot_URDFS/{self.robot_2}_joint_ID-{id}.urdf', encoding='utf-8', xml_declaration=True)
                break
            robot_1 = random.choice(os.listdir(root_path))
            robot_2 = random.choice(os.listdir(root_path))
            # print(robot_1)
            self.URDF_1 = ET.parse(root_path+f'{robot_1}/{robot_1}.urdf').getroot()
            self.URDF_2 = ET.parse(root_path+f'{robot_2}/{robot_2}.urdf').getroot()
            self.get_link_and_joint_info()
            # print(self.joint_axis_list_1,self.joint_axis_list_2)

if __name__ == '__main__':
    root_path = '../../URDFsForBoxi/'
    while len(os.listdir('../../Generated_robot_URDFS'))!= 50:
        while True:
            robot_1 = random.choice(os.listdir(root_path))
            robot_2 = random.choice(os.listdir(root_path))
            if robot_2 != robot_1:
                break
        swap = swap_legs_from_urdf(robot_1, robot_2)
        f_swap = ['link','joint']

        # while len(os.listdir('../../Generated_robot_URDFS'))!= 50:
        choice = random.choice(f_swap)
        if choice == 'link':
            swap.swap_links()
        if choice == 'joint':
            swap.swap_joints()

