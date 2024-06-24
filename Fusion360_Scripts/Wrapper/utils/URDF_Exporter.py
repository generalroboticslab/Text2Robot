#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os, shutil
from .utils import utils
from .utils import ExportYAML
from .core import Link, Joint, Write

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

def run(context, suffix, save_dir):
    ui = None
    success_msg = 'Successfully create URDF file'
    msg = success_msg
    
    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        
        title = 'Fusion2URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component 
        components = design.allComponents

        # for occ in rootComp.occurrences:
        #     if occ.isReferencedComponent:
        #         root = occ.component
        #         occ.breakLink()

        # for component in design.allComponents:
        #     if component != root and component != rootComp:
        #         components.add(component)

        

        # set the names        
        package_name = 'fusion2urdf'
        robot_name = root.name.split()[0] + suffix
        
        save_dir = save_dir + '/' + robot_name
        try: os.mkdir(save_dir)
        except: pass     

        # Export YAML File
        ExportYAML.run(context, save_dir, robot_name, root)
        
        # --------------------
        # set dictionaries
        
        # Generate joints_dict. All joints are related to root. 
        joints_dict, msg = Joint.make_joints_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0   
        
        # Generate inertial_dict
        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0
        
        links_xyz_dict = {}
        
        # --------------------
        # Generate URDF
        Write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, save_dir, robot_name, ui)
        Write.write_hello_pybullet(robot_name, save_dir)
        
        # Generate STl files
                
        utils.copy_occs(root)
        utils.export_stl(design, save_dir, components)   

        utils.simplify_meshes(save_dir)

        # save_dir + '/meshes'

        shutil.rmtree(save_dir + '/meshes')
        
        #ui.messageBox(msg, title)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
