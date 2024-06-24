import adsk.core, adsk.fusion, adsk.cam, traceback
import os
import yaml

def run(context, save_dir, robot_name, root):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        rootComp = root

        # FIND MIN HEIGHT
        min_height = 1000000 # initialize min_height to a big number
        for occ in rootComp.occurrences:
            for body in occ.bRepBodies:
                if occ.boundingBox.minPoint.z < min_height:
                    min_height = body.boundingBox.minPoint.z

        #ui.messageBox(str(min_height))

        # Collect all revolute joint names and set their angles to 0
        joint_angles = {}
        for joint in rootComp.joints:
            if joint.jointMotion.jointType == adsk.fusion.JointTypes.RevoluteJointType:
                joint_name = joint.name
                joint_angles[joint_name] = 0
        
        starting_position = -1 * (min_height / 100)
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
        
        filename = save_dir + '/' + robot_name + '.yaml'
        # Write data to a YAML file
        with open(filename, "w") as file:
            yaml.dump(data, file, default_flow_style=False)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))