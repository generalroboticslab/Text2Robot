#Author-
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback

from .utils import SliceBody2, URDF_Exporter, ScaleLimb
from .utils.utils import utils

import time
import os, sys
import subprocess


def run(context):
    
    
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        #ui.messageBox('Hello script')

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        root = design.rootComponent

        global organic
        organic = True

        global prismatic
        prismatic = False

        
        #_____________________________________________________________________________

        title = 'Fusion2URDF'
        # set the save directory
        save_dir = utils.file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0
        
        
        # slices single body into 9 bodies with joints
        SliceBody2.run(context)

        # creates extrusions for electronics
        createElectronicsBoxes()



        time.sleep(10)
       
        # # ui.messageBox("test")
        doc = app.activeDocument
        originalName = doc.name

        datafile = doc.dataFile

        name = root.name.split()[0]


        # Uncomment to export a single generated robot
        # URDF_Exporter.run(context, "test", save_dir)



        # Uncomment entire for loop to export 30 design variants

        # for i in range(6, 10):

        #     #time.sleep(5)
        #     #ui.messageBox("top of outer loop")
            

        #     translation_in_meters_right = adsk.core.Vector3D.create(0.00, 0.001, 0.0 - i*.005)  # Adjust the Z-value as needed.
        #     translation_in_meters_left = adsk.core.Vector3D.create(0.00, -0.001, 0 - i*.005)  # Adjust the Z-value as needed.
            
        #     translation_right = adsk.core.Vector3D.create(
        #         translation_in_meters_right.x * 100,
        #         translation_in_meters_right.y * 100,
        #         translation_in_meters_right.z * 100
        #     )

        #     translation_left = adsk.core.Vector3D.create(
        #         translation_in_meters_left.x * 100,
        #         translation_in_meters_left.y * 100,
        #         translation_in_meters_left.z * 100
        #     )


            

        #     #time.sleep(3)

        #     for j in range(3):
        #         if j == 0:
        #             setAllJoints("x")
        #             dir = "x"
        #         elif j == 1:
        #             setAllJoints("y")
        #             dir = "y"
        #         elif j == 2:
        #             setAllJoints("z")
        #             dir = "z"

        #         #time.sleep(5)
        #         ui.messageBox("about to scale")
        #         if i != 0:
        #             ScaleLimb.scaleWrapper(1, 0, translation_right)
        #             ScaleLimb.scaleWrapper(1, 1, translation_right)
        #             ScaleLimb.scaleWrapper(1, 2, translation_left)
        #             ScaleLimb.scaleWrapper(1, 3, translation_left)

        #             ui.messageBox("scaled top limbs")
        #             #time.sleep(5)

        #             ScaleLimb.scaleWrapper(2, 0, translation_right)
        #             ScaleLimb.scaleWrapper(2, 1, translation_right)
        #             ##ui.messageBox("pause")
        #             ScaleLimb.scaleWrapper(2, 2, translation_left)
        #             ScaleLimb.scaleWrapper(2, 3, translation_left)

        #             ui.messageBox("scaled bottom limbs")
        #             #time.sleep(5)

        #         createElectronicsBoxes()

                

        #         suffix = "_top-" + name + "-" + dir + "-" + str(i) + "_bottom-" + name + "-" + dir + "-" + str(i)

        #         #time.sleep(3)
        #         #ui.messageBox("about to export")
                

        #         URDF_Exporter.run(context, suffix, save_dir)

        #         #ui.messageBox("exported")
        #         #time.sleep(5)

        #         doc = app.activeDocument

        #         doc.close(False)

        #         time.sleep(1)

        #         app.documents.open(datafile)
        
        #___________________________________________________________________

    except:
        pass
        # if ui:
        #     ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def createElectronicsBoxes():
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    rootComp = design.rootComponent

    for occ in rootComp.allOccurrences:
            comp = occ.component
            
            #ui.messageBox(comp.name + "faces: "+ str(len(comp.bRepBodies[0].faces)))
            #ui.messageBox(comp.name)
            if comp.name == "body_1_0":
                body_1_0 = comp
                legPoint1 = SliceBody2.createLegPoint(comp)
                kneePoint1 = SliceBody2.createFootPoint(comp)
            elif comp.name == "body_1_1":
                body_1_1 = comp
                legPoint2 = SliceBody2.createLegPoint(comp)
                kneePoint2 = SliceBody2.createFootPoint(comp)
            elif comp.name == "body_1_2":
                body_1_2 = comp
                legPoint3 = SliceBody2.createLegPoint(comp)
                kneePoint3 = SliceBody2.createFootPoint(comp)
            elif comp.name == "body_1_3":
                body_1_3 = comp
                legPoint4 = SliceBody2.createLegPoint(comp)
                kneePoint4 = SliceBody2.createFootPoint(comp)
            elif comp.name == "body_2_0":
                body_2_0 = comp
                footPoint1 = SliceBody2.createFootPoint(comp)
            elif comp.name == "body_2_1":
                body_2_1 = comp
                footPoint2 = SliceBody2.createFootPoint(comp)
            elif comp.name == "body_2_2":
                body_2_2 = comp
                footPoint3 = SliceBody2.createFootPoint(comp)
            elif comp.name == "body_2_3":
                body_2_3 = comp
                footPoint4 = SliceBody2.createFootPoint(comp)
            elif comp.name == "base_link":
                base_link = comp
                bodyPoint1 = SliceBody2.createBodyPoint(comp, 1)
                bodyPoint2 = SliceBody2.createBodyPoint(comp, 2)
                bodyPoint3 = SliceBody2.createBodyPoint(comp, 3)
                bodyPoint4 = SliceBody2.createBodyPoint(comp, 4)

    motorBoxWidth = 4.44

    #ui.messageBox("pause")

    SliceBody2.createMotorBox(body_1_2, legPoint3, motorBoxWidth, motorBoxWidth, False)
    #ui.messageBox("pause")
    SliceBody2.createMotorBox(body_1_3, legPoint4, motorBoxWidth, motorBoxWidth, False)
    #ui.messageBox("pause")
    SliceBody2.createMotorBox(body_1_0, legPoint1, motorBoxWidth, motorBoxWidth, False)
    #ui.messageBox("pause")
    SliceBody2.createMotorBox(body_1_1, legPoint2, motorBoxWidth, motorBoxWidth, False)
    #ui.messageBox("pause")

    SliceBody2.createMotorBox(base_link, bodyPoint1, motorBoxWidth, motorBoxWidth, True)
    #ui.messageBox("pause")
    SliceBody2.createMotorBox(base_link, bodyPoint2, motorBoxWidth, motorBoxWidth, True)
    #ui.messageBox("pause")
    SliceBody2.createMotorBox(base_link, bodyPoint3, motorBoxWidth, motorBoxWidth, True)
    #ui.messageBox("pause")
    SliceBody2.createMotorBox(base_link, bodyPoint4, motorBoxWidth, motorBoxWidth, True)

    #ui.messageBox("pause")

    SliceBody2.createKneeBox(body_1_0, kneePoint1, motorBoxWidth, motorBoxWidth, True)
    #ui.messageBox("pause")
    SliceBody2.createKneeBox(body_1_1, kneePoint2, motorBoxWidth, motorBoxWidth, True)
    #ui.messageBox("pause")
    SliceBody2.createKneeBox(body_1_2, kneePoint3, motorBoxWidth, motorBoxWidth, True)
    #ui.messageBox("pause")
    SliceBody2.createKneeBox(body_1_3, kneePoint4, motorBoxWidth, motorBoxWidth, True)

    #ui.messageBox("pause")

    SliceBody2.createKneeBox(body_2_0, footPoint1, motorBoxWidth, motorBoxWidth, False)
    #ui.messageBox("pause")
    SliceBody2.createKneeBox(body_2_1, footPoint2, motorBoxWidth, motorBoxWidth, False)
    #ui.messageBox("pause")
    SliceBody2.createKneeBox(body_2_2, footPoint3, motorBoxWidth, motorBoxWidth, False)
    #ui.messageBox("pause")
    SliceBody2.createKneeBox(body_2_3, footPoint4, motorBoxWidth, motorBoxWidth, False)

    #ui.messageBox("pause")


    SliceBody2.createElectronicsBox(base_link, 7.05, 12.85, -legPoint1.geometry.y)
    #ui.messageBox("pause")




    


def setAllJoints(dir):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    rootComp = design.rootComponent

    #ui.messageBox("in set all joints")

    for j in rootComp.joints:
        mutateJoint(j.name, dir)


def mutateJoint(jointName, dir):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    rootComp = design.rootComponent

    #ui.messageBox("changing joint")

    j = rootComp.joints.itemByName(jointName)

    revolute_joint = adsk.fusion.RevoluteJointMotion.cast(j.jointMotion)
    jointAxis = revolute_joint.rotationAxis

    if dir == "y":
        j.jointMotion.rotationAxis = adsk.fusion.JointDirections.YAxisJointDirection

    elif dir == "z":
        j.jointMotion.rotationAxis = adsk.fusion.JointDirections.ZAxisJointDirection

    elif dir == "x":
        j.jointMotion.rotationAxis = adsk.fusion.JointDirections.XAxisJointDirection



def fork_and_export(context, suffix, save_dir):


    app = adsk.core.Application.get()
    ui  = app.userInterface

    
    time.sleep(10)
    

    parentProduct = app.activeProduct
    

    parentDesign = adsk.fusion.Design.cast(parentProduct)

    parentRootComp = parentDesign.rootComponent

    docParent = app.activeDocument

    occs = parentRootComp.occurrences
    
    # ui.messageBox("isUpToDate: "+str(docParent.isUpToDate))
    # ui.messageBox("isSaved: "+str(docParent.isSaved))
    # ui.messageBox("isModified: "+str(docParent.isModified))
    #docParent.updateAllReferences()

    
    
    # temp_file_path = adsk.fusion.SystemUtil.tempFolder + parentDesign.name

    while not docParent.isSaved:
        pass
    

    docChild = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)

    childProduct = app.activeProduct
    

    childDesign = adsk.fusion.Design.cast(childProduct)

    childRootComp = childDesign.rootComponent

    

    childRootComp.occurrences.addByInsert(docParent.dataFile, adsk.core.Matrix3D.create(), True)

    # for occ in occs:

    #     childRootComp.occurrences.addExistingComponent(occ, adsk.core.Matrix3D.create())

    #ui.messageBox("success addbyinsert: "+str(success))

    


    
    


    #ui.messageBox("Call ELM, Export URDF, then close")

    #suffix = "_scaleNum_jointDir"

    URDF_Exporter.run(context, suffix, save_dir)

    #docChild.close(False)