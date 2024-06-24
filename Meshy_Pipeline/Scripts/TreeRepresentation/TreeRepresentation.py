import adsk.core, adsk.fusion, adsk.cam, traceback
import os

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        rootComp = design.rootComponent

        # TODO: Update directory_path to where your specific GenerateRobot folder is stored
        directory_path = 'C:/Users/zacha/OneDrive/Documents/GRL/RobotsMakingRobots/Meshy Pipeline/Scripts/TreeRepresentation'
        os.chdir(directory_path)

        data_to_write_to_file = ''

        for occ in rootComp.occurrences:
            data_to_write_to_file = data_to_write_to_file + occ.name + '\n'
        
        yaml_filename = "test"
        filename = f"Trees/{yaml_filename}.txt"
        
        with open(filename, 'w') as file:
            file.write(data_to_write_to_file)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))