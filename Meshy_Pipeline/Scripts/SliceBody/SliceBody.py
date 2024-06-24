#Author-
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback
import numpy as np
from scipy.signal import argrelextrema


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        design.designType = adsk.fusion.DesignTypes.DirectDesignType

        # Get the root component of the active design
        rootComp = design.rootComponent

        # Create sketch
        sketches = rootComp.sketches

        planes = rootComp.constructionPlanes
        axes = rootComp.constructionAxes

        
        

        ui.messageBox("running")


        #get Cross section Data________________________________________________________________
        crossarr = np.genfromtxt("C:/Users/zacha/Desktop/Cross Sections/cross_" + str(app.activeDocument.dataFile.name) + ".csv", delimiter=",")

        yzarr = crossarr[:, 0]
        xzarr = crossarr[:, 1]
        xyarr = crossarr[:, 2]
        inx = crossarr[:, 3]

        #find min of xzarr
        minima = argrelextrema(xzarr,np.less)
        maxima = argrelextrema(xzarr, np.greater)

        #find closest minima to center of body, impose symmetry, slice body

        xzminima = inx[minima]
        xzmaxima = inx[maxima]

        possibleOffsets = abs(xzminima)
        #possibleOffsets = [0.0]

        #set threshold volume as 10% of initial total volume
        global thresholdVol
        thresholdVol = 0.02 * design.rootComponent.bRepBodies[0].volume 

        offsetShoulderList = np.sort(possibleOffsets)

        #Todo: Add logic to compute offset as a list of possible points to iterate through in subsequent code************

        


        #create planes to split bodies_________________________________________________________


        number_of_legs = 4
        design.rootComponent.bRepBodies[0].name = "base_link"



        mainbody = design.rootComponent.bRepBodies.itemByName("base_link")
        targetBodyNum = len(design.rootComponent.bRepBodies) + 2 #Shoulder slice for quadruped should yield two legs + main body

        #testList = [0]

        #change to offset[0]
        correctOffset, leg1, leg2, correctPlane = sliceForShoulder(mainbody, offsetShoulderList, targetBodyNum)

        ui.messageBox(str(correctOffset))

        leg1.name = "leg1"
        leg2.name = "leg2"
        
            
        #create joints for prev slice
        createShoulderJoints(leg1, leg2, mainbody, correctPlane)

        

        mainbody = design.rootComponent.bRepBodies.itemByName("base_link")
        targetBodyNum = len(design.rootComponent.bRepBodies) + 2 #Shoulder slice for quadruped should yield two legs + main body

        symList = np.zeros(1)
        symList[0] = -correctOffset

        symOffset, leg3, leg4, symPlane = sliceForShoulder(mainbody, symList, targetBodyNum)

        leg3.name = "leg3"
        leg4.name = "leg4"

        createShoulderJoints(leg3, leg4, mainbody, symPlane)
        

        #Create base_link as component and set up joints for urdf exportation

        mainbody = design.rootComponent.bRepBodies.itemByName("base_link")
        base_link = rootComp.parentDesign.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
        base_link.bRepBodies.add(mainbody)
        base_link.name = "base_link"
        mainbody.deleteMe()

        #set joints somehow

        for joint in rootComp.joints:
            joint.occurrenceTwo = rootComp.occurrences[len(rootComp.occurrences)-1]
        

        

        #__________________________________________________________________

        
        


        


        #create knee joint ___________________________________



        
        
        




    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))



def create_component_from_body(existing_body):
    app = adsk.core.Application.get()
    design = app.activeProduct

    # Get the root component of the active design
    root_comp = design.rootComponent

    # Create a new component
    new_comp = root_comp.parentDesign.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component

    # Add the existing body to the new component
    new_comp.bRepBodies.add(existing_body)

    return new_comp


def sliceForShoulder(mainbody, offset, targetBodyNum):
    app = adsk.core.Application.get()
    design = app.activeProduct
    rootComp = design.rootComponent
    ui  = app.userInterface

    planes = rootComp.constructionPlanes
    axes = rootComp.constructionAxes
    sketches = rootComp.sketches

    rotated = False
    offsetResult = offset[0]

    for i in range(len(offset)):

        #create planes to slice legs
        planeInput = planes.createInput()
        offsetValue = adsk.core.ValueInput.createByReal(offset[i])
        planeInput.setByOffset(rootComp.xZConstructionPlane, offsetValue)
        planeOne = planes.add(planeInput)

       

        
        splitBodyFeats = rootComp.features.splitBodyFeatures
        splitBodyInput = splitBodyFeats.createInput(mainbody, planeOne, True)
        splitBodyFeats.add(splitBodyInput)


        #store number of bodies > threshold vol
        validBodies = 0
        for body in design.rootComponent.bRepBodies:
            if body.volume > thresholdVol:
                validBodies = validBodies+1
        

        #after first slice there should be 3 bodies, if not 3, undo slice and iterate outward to find ideal slice
        #Todo*************
        if validBodies != targetBodyNum:
            ui.messageBox("incorrect body slice, do better, validbodies: "+str(validBodies) + " targetBodyNum: "+str(targetBodyNum))
            #write some good logic

            #undo slice and rotate plane: undo slice and try different offset, etc
            planeOne.deleteMe()
            if len(design.rootComponent.bRepBodies) == 1:
                
                continue
            else:
                
                features = design.rootComponent.features

                combineFeatures = features.combineFeatures
                targetBody = design.rootComponent.bRepBodies[0]
                toolBodies = adsk.core.ObjectCollection.create()

                for j in range(1, len(design.rootComponent.bRepBodies)):
                    toolBodies.add(design.rootComponent.bRepBodies[j])
                
                combineInput = combineFeatures.createInput(targetBody, toolBodies)

                combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation

                combineInput.isKeepToolBody = False

                combineFeature = combineFeatures.add(combineInput)

                design.rootComponent.bRepBodies[0].name = "base_link"

        else:
            if validBodies != len(design.rootComponent.bRepBodies):
                #Undo then try planes at different angles
                features = design.rootComponent.features

                combineFeatures = features.combineFeatures
                targetBody = design.rootComponent.bRepBodies[0]
                toolBodies = adsk.core.ObjectCollection.create()

                for j in range(1, len(design.rootComponent.bRepBodies)):
                    toolBodies.add(design.rootComponent.bRepBodies[j])
                
                combineInput = combineFeatures.createInput(targetBody, toolBodies)
                combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
                combineInput.isKeepToolBody = False
                combineFeature = combineFeatures.add(combineInput)
                design.rootComponent.bRepBodies[0].name = "base_link"

                #create axis at offset[i]

                # Create the axis input
                axis_input = axes.createInput()
                axis_input.setByTwoPlanes(rootComp.xYConstructionPlane, planeOne)

                # Create the axis
                axis = axes.add(axis_input)

                
                ui.messageBox("trying angles now")
                
                #loop through all angles and try to slice
                for k in range(360):
                    planeInput = planes.createInput()
                    angle = adsk.core.ValueInput.createByReal(k*0.0174533)
                    planeInput.setByAngle(axis, angle, planeOne)
                    anglePlane = planes.add(planeInput)

                    splitBodyFeats = rootComp.features.splitBodyFeatures
                    splitBodyInput = splitBodyFeats.createInput(mainbody, anglePlane, True)
                    splitBodyFeats.add(splitBodyInput)

                    if len(design.rootComponent.bRepBodies) == targetBodyNum:
                        rotated = True
                        break
                    else:
                        #undo and try next angle
                        anglePlane.deleteMe()
                        features = design.rootComponent.features

                        combineFeatures = features.combineFeatures
                        targetBody = design.rootComponent.bRepBodies[0]
                        toolBodies = adsk.core.ObjectCollection.create()

                        for j in range(1, len(design.rootComponent.bRepBodies)):
                            toolBodies.add(design.rootComponent.bRepBodies[j])
                        
                        combineInput = combineFeatures.createInput(targetBody, toolBodies)
                        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
                        combineInput.isKeepToolBody = False
                        combineFeature = combineFeatures.add(combineInput)
                        design.rootComponent.bRepBodies[0].name = "base_link"


            offsetResult = offset[i]
            break

        

    #Case where none of the special points created the correct number of bodies____________________
    if len(design.rootComponent.bRepBodies) is not targetBodyNum:
        ui.messageBox("incorrect body slice, all possible offsets expended, do better")
        #loop from plane offset = 0 until 2 legs detected

        for i in range(0, 100, 2):
            planeInput = planes.createInput()
            offsetValue = adsk.core.ValueInput.createByReal(i/10)
            planeInput.setByOffset(rootComp.xZConstructionPlane, offsetValue)
            planeOne = planes.add(planeInput)

            

            
            splitBodyFeats = rootComp.features.splitBodyFeatures
            splitBodyInput = splitBodyFeats.createInput(mainbody, planeOne, True)
            splitBodyFeats.add(splitBodyInput)

            
            validBodies = 0
            for body in design.rootComponent.bRepBodies:
                if body.volume > thresholdVol:
                    validBodies = validBodies+1


            #after first slice there should be 3 bodies, if not 3, undo slice and iterate outward to find ideal slice
            #Todo*************
            if validBodies != targetBodyNum:
                ui.messageBox("incorrect body slice, do better, validbodies: "+str(validBodies) + " targetBodyNum: "+str(targetBodyNum))
                #write some good logic

                #undo slice and rotate plane: undo slice and try different offset, etc
                planeOne.deleteMe()
                if len(design.rootComponent.bRepBodies) == 1:
                    
                    continue
                else:
                    
                    features = design.rootComponent.features

                    combineFeatures = features.combineFeatures
                    targetBody = design.rootComponent.bRepBodies[0]
                    toolBodies = adsk.core.ObjectCollection.create()

                    for j in range(1, len(design.rootComponent.bRepBodies)):
                        toolBodies.add(design.rootComponent.bRepBodies[j])

                    combineInput = combineFeatures.createInput(targetBody, toolBodies)

                    combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation

                    combineInput.isKeepToolBody = False

                    combineFeature = combineFeatures.add(combineInput)

                    design.rootComponent.bRepBodies[0].name = "base_link"

            else:
                if validBodies != len(design.rootComponent.bRepBodies):
                    #Undo then try planes at different angles
                    features = design.rootComponent.features

                    combineFeatures = features.combineFeatures
                    targetBody = design.rootComponent.bRepBodies[0]
                    toolBodies = adsk.core.ObjectCollection.create()

                    for j in range(1, len(design.rootComponent.bRepBodies)):
                        toolBodies.add(design.rootComponent.bRepBodies[j])
                    
                    combineInput = combineFeatures.createInput(targetBody, toolBodies)
                    combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
                    combineInput.isKeepToolBody = False
                    combineFeature = combineFeatures.add(combineInput)
                    design.rootComponent.bRepBodies[0].name = "base_link"

                    #create axis at offset[i]

                    # Create the axis input
                    axis_input = axes.createInput()
                    axis_input.setByTwoPlanes(rootComp.xYConstructionPlane, planeOne)

                    # Create the axis
                    axis = axes.add(axis_input)

                    
                    ui.messageBox("trying angles now")
                    
                    #loop through all angles and try to slice
                    for k in range(360):
                        planeInput = planes.createInput()
                        angle = adsk.core.ValueInput.createByReal(k*0.0174533)
                        planeInput.setByAngle(axis, angle, planeOne)
                        anglePlane = planes.add(planeInput)

                        splitBodyFeats = rootComp.features.splitBodyFeatures
                        splitBodyInput = splitBodyFeats.createInput(mainbody, anglePlane, True)
                        splitBodyFeats.add(splitBodyInput)

                        if len(design.rootComponent.bRepBodies) == targetBodyNum:
                            rotated = True
                            break
                        else:
                            #undo and try next angle
                            anglePlane.deleteMe()
                            features = design.rootComponent.features

                            combineFeatures = features.combineFeatures
                            targetBody = design.rootComponent.bRepBodies[0]
                            toolBodies = adsk.core.ObjectCollection.create()

                            for j in range(1, len(design.rootComponent.bRepBodies)):
                                toolBodies.add(design.rootComponent.bRepBodies[j])
                            
                            combineInput = combineFeatures.createInput(targetBody, toolBodies)
                            combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
                            combineInput.isKeepToolBody = False
                            combineFeature = combineFeatures.add(combineInput)
                            design.rootComponent.bRepBodies[0].name = "base_link"


                offsetResult = i/10
                break

        #swap order of legs somehow
        maxVol = 0
        for body in design.rootComponent.bRepBodies:
            if body.volume > maxVol:
                maxVol = body.volume

        i = 2
        for body in design.rootComponent.bRepBodies:
            if body.volume == maxVol:
                body.name = "base_link"
            else:
                body.name = "leg_" + str(i)
                i = i-1

    else:
        maxVol = 0
        for body in design.rootComponent.bRepBodies:
            if body.volume > maxVol:
                maxVol = body.volume

        i = 1
        for body in design.rootComponent.bRepBodies:
            if body.volume == maxVol:
                body.name = "base_link"
            else:
                body.name = "leg_" + str(i)
                i = i+1


    
            

    leg_1 = design.rootComponent.bRepBodies.itemByName("leg_1")
    leg_2 = design.rootComponent.bRepBodies.itemByName("leg_2")

    #create components from leg bodies___________________
    leg1 = rootComp.parentDesign.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
    leg1.bRepBodies.add(leg_1)
    #leg1.name = "leg1"
    leg_1.deleteMe()
    
    leg2 = rootComp.parentDesign.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
    leg2.bRepBodies.add(leg_2)
    #leg2.name = "leg2"
    leg_2.deleteMe()

    if rotated:
        correctPlane = anglePlane
    else:
        planeInput = planes.createInput()
        offsetValue = adsk.core.ValueInput.createByReal(offsetResult)
        planeInput.setByOffset(rootComp.xZConstructionPlane, offsetValue)
        correctPlane = planes.add(planeInput)

    return offsetResult, leg1, leg2, correctPlane

    
def createShoulderJoints(leg1, leg2, mainbody, correctPlane):
    app = adsk.core.Application.get()
    design = app.activeProduct
    rootComp = design.rootComponent
    sketches = rootComp.sketches
    ui  = app.userInterface

    planes = rootComp.constructionPlanes

    #ui.messageBox(design.rootComponent.bRepBodies[0].name)

    #create plane shoulder was sliced at 
    

    #create 3 sketches, 4 points: one for each leg, 1 sketch w 2 points for main body

    #slice base_link body and get sketch profile
    sketch = sketches.add(correctPlane)

    entities = []
    entities.append(design.rootComponent.bRepBodies.itemByName("base_link"))
    

    sketch.intersectWithSketchPlane(entities)

    boundingBox = sketch.profiles[0].boundingBox
    minPoint = boundingBox.minPoint
    maxPoint = boundingBox.maxPoint

    centerX = (minPoint.x + maxPoint.x) / 2
    centerY = (minPoint.y + maxPoint.y) / 2
    centerZ = (minPoint.z + maxPoint.z) / 2

    # The approximate center point
    centerPoint1 = adsk.core.Point3D.create(centerX, centerY, centerZ)
    bodyPoint1 = sketch.sketchPoints.add(centerPoint1)

    boundingBox = sketch.profiles[1].boundingBox
    minPoint = boundingBox.minPoint
    maxPoint = boundingBox.maxPoint

    centerX = (minPoint.x + maxPoint.x) / 2
    centerY = (minPoint.y + maxPoint.y) / 2
    centerZ = (minPoint.z + maxPoint.z) / 2

    # The approximate center point
    centerPoint2 = adsk.core.Point3D.create(centerX, centerY, centerZ)
    bodyPoint2 = sketch.sketchPoints.add(centerPoint2)

    
    
    sketches2 = leg1.sketches

    sketch2 = sketches2.add(correctPlane)
    entities = []
    entities.append(leg1.bRepBodies[0])

    sketch2.intersectWithSketchPlane(entities)

    boundingBox = sketch2.profiles[0].boundingBox
    minPoint = boundingBox.minPoint
    maxPoint = boundingBox.maxPoint

    centerX = (minPoint.x + maxPoint.x) / 2
    centerY = (minPoint.y + maxPoint.y) / 2
    centerZ = (minPoint.z + maxPoint.z) / 2

    # The approximate center point
    centerPoint3 = adsk.core.Point3D.create(centerX, centerY, centerZ)
    leg1Point = sketch2.sketchPoints.add(centerPoint3)

    #_____________________ joint for second leg
    sketches3 = leg2.sketches

    sketch3 = sketches3.add(correctPlane)
    entities = []
    entities.append(leg2.bRepBodies[0])

    sketch3.intersectWithSketchPlane(entities)

    boundingBox = sketch3.profiles[0].boundingBox
    minPoint = boundingBox.minPoint
    maxPoint = boundingBox.maxPoint

    centerX = (minPoint.x + maxPoint.x) / 2
    centerY = (minPoint.y + maxPoint.y) / 2
    centerZ = (minPoint.z + maxPoint.z) / 2

    # The approximate center point
    centerPoint4 = adsk.core.Point3D.create(centerX, centerY, centerZ)
    leg2Point = sketch3.sketchPoints.add(centerPoint4)

    
    
    # Create the second joint geometry with prof1
    geo1 = adsk.fusion.JointGeometry.createByPoint(bodyPoint1)
    geo4 = adsk.fusion.JointGeometry.createByPoint(bodyPoint2)

    # Create the first joint geometry with the side face
    geo2 = adsk.fusion.JointGeometry.createByPoint(leg1Point)
    geo3 = adsk.fusion.JointGeometry.createByPoint(leg2Point)
        
    #create one joint
    joints = rootComp.joints
    
    jointInput = joints.createInput(geo2, geo1)

    jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    joint1 = joints.add(jointInput)
    
    revoluteMotion = joint1.jointMotion

    jointInput = joints.createInput(geo3, geo4)

    jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    joint2 = joints.add(jointInput)
    
    revoluteMotion = joint2.jointMotion

    #ui.messageBox("occ1: "+joint1.occurrenceOne.component.name +"occ2: " + joint1.occurrenceTwo.component.name)
