#Author-
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback
import numpy as np
from scipy.signal import argrelextrema
import math



def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        rootComp = design.rootComponent
        scales = rootComp.features.scaleFeatures

        #Set initial threshold volume, define mainbody as initial single body
        design.rootComponent.bRepBodies[0].name = "base_link"
        mainbody = design.rootComponent.bRepBodies.itemByName("base_link")

        #set custom material for a custom density
        existingMaterial = app.favoriteMaterials.itemByName("Polyethylene, Low Density")

        if app.favoriteMaterials.itemByName('My ABS Plastic') == None:

            newMaterial = design.materials.addByCopy(existingMaterial, 'My ABS Plastic')
            densityProp: adsk.core.FloatProperty = newMaterial.materialProperties.itemByName('Density')
            densityProp.value = 380.79

        else:
            newMaterial = app.favoriteMaterials.itemByName('My ABS Plastic')


        app.preferences.materialPreferences.defaultMaterial = newMaterial

        
        #set initial threshhold volume
        global thresholdVol
        thresholdVol = 0.02 * mainbody.volume 

        #define body as organic conversion
        global organic
        organic = True

        global prismatic
        prismatic = False

        motorWidth = 4.44
        motorLength = 4.44

        initialVolume = mainbody.volume
        #hard coded targetVolume in cm^3
        targetVolume = 6300


        scaleNum2 = math.cbrt(targetVolume/initialVolume)
        scaleFactor = adsk.core.ValueInput.createByReal(scaleNum2)
        

        #First, slice single body into 5 bodies, 4 legs and one mainbody

        # Adjust steps to determine how close to body the plane should slice: 
        # larger is closer, smaller is farther
        sliced, cleanCut, correctOffset = sliceByDX(mainbody, 10, False, 3)
        

        #3 bodies exist if sliced
        if len(rootComp.bRepBodies) != 3:
            
            #retry fuseBodies 
            validBodies = adsk.core.ObjectCollection.create()
            invalidBodies = adsk.core.ObjectCollection.create()
            for body in rootComp.bRepBodies:
                if body.volume > thresholdVol:
                    validBodies.add(body)
                else:
                    invalidBodies.add(body)
            fuseInvalid(validBodies, invalidBodies)

        if len(rootComp.bRepBodies) != 3:
            
            #retry fuseBodies 
            validBodies = adsk.core.ObjectCollection.create()
            invalidBodies = adsk.core.ObjectCollection.create()
            for body in rootComp.bRepBodies:
                if body.volume > thresholdVol:
                    validBodies.add(body)
                else:
                    invalidBodies.add(body)
            fuseInvalid(validBodies, invalidBodies)

        if sliced:
            
            body1 = rootComp.bRepBodies[0]
            body2 = rootComp.bRepBodies[1]
            body3 = rootComp.bRepBodies[2]

            #figure out names of bodies as base_link, body_1_0, and body_1_1 from position
            bbox1 = body1.boundingBox
            bbox2 = body2.boundingBox
            bbox3 = body3.boundingBox

            if bbox1.minPoint.y < 0:
                #body1 is base_link/mainbody
                body1.name = "base_link"
                mainbody = body1

                if bbox2.maxPoint.x > bbox3.maxPoint.x:
                    #body2 is body_1_0, body3 is body_1_1
                    body2.name = "body_1_0"
                    body3.name = "body_1_1"
                else:
                    body3.name = "body_1_0"
                    body2.name = "body_1_1"
            elif bbox2.minPoint.y < 0:
                body2.name = "base_link"
                mainbody = body2

                if bbox1.maxPoint.x > bbox3.maxPoint.x:
                    #body2 is body_1_0, body3 is body_1_1
                    body1.name = "body_1_0"
                    body3.name = "body_1_1"
                else:
                    body3.name = "body_1_0"
                    body1.name = "body_1_1"
            elif bbox3.minPoint.y < 0:
                body3.name = "base_link"
                mainbody = body3

                if bbox1.maxPoint.x > bbox2.maxPoint.x:
                    #body2 is body_1_0, body3 is body_1_1
                    body1.name = "body_1_0"
                    body2.name = "body_1_1"
                else:
                    body2.name = "body_1_0"
                    body1.name = "body_1_1"
                
        
        symoffsets = [correctOffset]
        sliced, cleanCut, correctOffset = sliceByMinima(mainbody, symoffsets, True, 5)

        if sliced == False:
            sliced, cleanCut, correctOffset = sliceByDX(mainbody, 10, True, 5)

        #5 bodies exist if sliced  
        if len(rootComp.bRepBodies) != 5:
                ui.messageBox("ERROR: slice created wrong number of bodies, retrying fuse")
                #retry fusebodies
                
                validBodies = adsk.core.ObjectCollection.create()
                invalidBodies = adsk.core.ObjectCollection.create()
                for body in rootComp.bRepBodies:
                    if body.volume > thresholdVol:
                        validBodies.add(body)
                    else:
                        invalidBodies.add(body)
                fuseInvalid(validBodies, invalidBodies)

        if len(rootComp.bRepBodies) != 5:
                ui.messageBox("ERROR: slice created wrong number of bodies, retrying fuse")
                #retry fusebodies
                
                validBodies = adsk.core.ObjectCollection.create()
                invalidBodies = adsk.core.ObjectCollection.create()
                for body in rootComp.bRepBodies:
                    if body.volume > thresholdVol:
                        validBodies.add(body)
                    else:
                        invalidBodies.add(body)
                fuseInvalid(validBodies, invalidBodies)

        if sliced:
            
            i = 0
            for body in rootComp.bRepBodies:
                if body.name != "body_1_0" and body.name != "body_1_1":
                    i = i+1
                    if i == 1:
                        body1 = body
                    elif i == 2:
                        body2 = body
                    elif i == 3:
                        body3 = body
            
            bbox1 = body1.boundingBox
            bbox2 = body2.boundingBox
            bbox3 = body3.boundingBox

            if bbox1.maxPoint.y > 0:
                #body1 is base_link/mainbody
                body1.name = "base_link"
                mainbody = body1

                if bbox2.maxPoint.x > bbox3.maxPoint.x:
                    #body2 is body_1_0, body3 is body_1_1
                    body2.name = "body_1_2"
                    body3.name = "body_1_3"
                else:
                    body3.name = "body_1_2"
                    body2.name = "body_1_3"
            elif bbox2.maxPoint.y > 0:
                body2.name = "base_link"
                mainbody = body2

                if bbox1.maxPoint.x > bbox3.maxPoint.x:
                    #body2 is body_1_0, body3 is body_1_1
                    body1.name = "body_1_2"
                    body3.name = "body_1_3"
                else:
                    body3.name = "body_1_2"
                    body1.name = "body_1_3"
            elif bbox3.maxPoint.y > 0:
                body3.name = "base_link"
                mainbody = body3

                if bbox1.maxPoint.x > bbox2.maxPoint.x:
                    #body2 is body_1_0, body3 is body_1_1
                    body1.name = "body_1_2"
                    body2.name = "body_1_3"
                else:
                    body2.name = "body_1_2"
                    body1.name = "body_1_3"

        
        #compute scale factor based on 8 faces
        # scaleNum = scaleFactorFromFaces(motorWidth*1.25, motorLength*1.25) # ignore depth for now...
        # ui.messageBox("scaleNum1 = " + str(scaleNum) + "scaleNum2 = "+ str(scaleNum2))
        # scaleNum = max(scaleNum, scaleNum2)
        # #ui.messageBox("chosen scaleNum = "+str(scaleNum))
        # #scale bodies
        # scaleFactor = adsk.core.ValueInput.createByReal(scaleNum)

        inputColl = adsk.core.ObjectCollection.create()
        

        #add components
        for body in rootComp.bRepBodies:
            #ui.messageBox(occ.component.name)
            inputColl.add(body)

        

        scaleInput = scales.createInput(inputColl, rootComp.originConstructionPoint, scaleFactor)
        
        #ui.messageBox("pre-scale, scaleFactor = "+str(scaleFactor))
        scale = scales.add(scaleInput)

        #ui.messageBox("pause")
        #slice knee joint: identify plane somehow, then slice: add points, offset points then join points

        sliced = sliceKnees(rootComp)
        
        #ui.messageBox("pause")
        #convert each leg into a component
        body_1_0: adsk.fusion.Component = create_component_from_body(rootComp.bRepBodies.itemByName("body_1_0"))
        body_1_1: adsk.fusion.Component = create_component_from_body(rootComp.bRepBodies.itemByName("body_1_1"))
        body_1_0.name = "body_1_0"
        body_1_1.name = "body_1_1"
        
        
        body_1_2: adsk.fusion.Component = create_component_from_body(rootComp.bRepBodies.itemByName("body_1_2"))
        body_1_3: adsk.fusion.Component = create_component_from_body(rootComp.bRepBodies.itemByName("body_1_3"))
        body_1_2.name = "body_1_2"
        body_1_3.name = "body_1_3"

        #convert feet to components
        body_2_0 = create_component_from_body(rootComp.bRepBodies.itemByName("body_2_0"))
        body_2_1 = create_component_from_body(rootComp.bRepBodies.itemByName("body_2_1"))
        body_2_2 = create_component_from_body(rootComp.bRepBodies.itemByName("body_2_2"))
        body_2_3 = create_component_from_body(rootComp.bRepBodies.itemByName("body_2_3"))
        body_2_0.name = "body_2_0"
        body_2_1.name = "body_2_1"
        body_2_2.name = "body_2_2"
        body_2_3.name = "body_2_3"
        


        #set base_link as component
        base_link = create_component_from_body(rootComp.bRepBodies[0])
        base_link.name = "base_link"
        base_link.isGrounded = True

        #ui.messageBox("pause")

        #create joints between components______________________________________________

        #create sketches on each leg, and mainbody, to create points for joint geometry

        #find flat face for each

        for occ in rootComp.allOccurrences:
            comp = occ.component
            
            #ui.messageBox(comp.name + "faces: "+ str(len(comp.bRepBodies[0].faces)))
            #ui.messageBox(comp.name)
            if comp.name == "body_1_0":
                legPoint1 = createLegPoint(comp)
                kneePoint1 = createFootPoint(comp)
            elif comp.name == "body_1_1":
                legPoint2 = createLegPoint(comp)
                kneePoint2 = createFootPoint(comp)
            elif comp.name == "body_1_2":
                legPoint3 = createLegPoint(comp)
                kneePoint3 = createFootPoint(comp)
            elif comp.name == "body_1_3":
                legPoint4 = createLegPoint(comp)
                kneePoint4 = createFootPoint(comp)
            elif comp.name == "body_2_0":
                footPoint1 = createFootPoint(comp)
            elif comp.name == "body_2_1":
                footPoint2 = createFootPoint(comp)
            elif comp.name == "body_2_2":
                footPoint3 = createFootPoint(comp)
            elif comp.name == "body_2_3":
                footPoint4 = createFootPoint(comp)
            elif comp.name == "base_link":
                bodyPoint1 = createBodyPoint(comp, 1)
                #ui.messageBox("pause")
                bodyPoint2 = createBodyPoint(comp, 2)
                #ui.messageBox("pause")
                bodyPoint3 = createBodyPoint(comp, 3)
                #ui.messageBox("pause")
                bodyPoint4 = createBodyPoint(comp, 4)

            #ui.messageBox("pause")

        #ui.messageBox("pause")

        #create scale factor
        # todo: offset the points at a distance depending on type of joint
                
        legPoint1Off = offsetPoint(body_1_0, legPoint1, -2, "y")
        #ui.messageBox("pause")
        legPoint2Off = offsetPoint(body_1_1, legPoint2, -2, "y")
        #ui.messageBox("pause")
        legPoint3Off = offsetPoint(body_1_2, legPoint3, 2, "y")
        #ui.messageBox("pause")
        legPoint4Off = offsetPoint(body_1_3, legPoint4, 2, "y")

        #ui.messageBox("pause")

        kneePoint1Off = offsetPoint(body_1_0, kneePoint1, -2, "z")
        #ui.messageBox("pause")
        kneePoint2Off = offsetPoint(body_1_1, kneePoint2, -2, "z")
        #ui.messageBox("pause")
        kneePoint3Off = offsetPoint(body_1_2, kneePoint3, -2, "z")
        #ui.messageBox("pause")
        kneePoint4Off = offsetPoint(body_1_3, kneePoint4, -2, "z")

        #ui.messageBox("pause")

        footPoint1Off = offsetPoint(body_2_0, footPoint1, 2, "z")
        #ui.messageBox("pause")
        footPoint2Off = offsetPoint(body_2_1, footPoint2, 2, "z")
        #ui.messageBox("pause")
        footPoint3Off = offsetPoint(body_2_2, footPoint3, 2, "z")
        #ui.messageBox("pause")
        footPoint4Off = offsetPoint(body_2_3, footPoint4, 2, "z")

        #ui.messageBox("pause")
        
        bodyPoint1Off = offsetPoint(base_link, bodyPoint1, 2, "y")
        #ui.messageBox("pause")
        bodyPoint2Off = offsetPoint(base_link, bodyPoint2, 2, "y")
        #ui.messageBox("pause")
        bodyPoint3Off = offsetPoint(base_link, bodyPoint3, -2, "y")
        #ui.messageBox("pause")
        bodyPoint4Off = offsetPoint(base_link, bodyPoint4, -2, "y")
        
        #ui.messageBox("pause")
        #create joints from points
        joint_1_0 = joinPoints(legPoint1Off, bodyPoint1Off, "z")   
        joint_1_0.name = "joint_1_0"
        #ui.messageBox("pause")
        joint_1_1 = joinPoints(legPoint2Off, bodyPoint2Off, "z") 
        joint_1_1.name = "joint_1_1"
        #ui.messageBox("pause")
        joint_1_2 = joinPoints(legPoint3Off, bodyPoint3Off, "z")
        joint_1_2.name = "joint_1_2" 
        #ui.messageBox("pause")
        joint_1_3 = joinPoints(legPoint4Off, bodyPoint4Off, "z")    
        joint_1_3.name = "joint_1_3"

        #ui.messageBox("pause")

        #next is zz
        joint_2_0 = joinPoints(footPoint1Off, kneePoint1Off, "z")   
        joint_2_0.name = "joint_2_0"
        #ui.messageBox("pause")
        joint_2_1 = joinPoints(footPoint2Off, kneePoint2Off, "z")
        joint_2_1.name = "joint_2_1"
        #ui.messageBox("pause")
        joint_2_2 = joinPoints(footPoint3Off, kneePoint3Off, "z")
        joint_2_2.name = "joint_2_2"
        #ui.messageBox("pause")
        joint_2_3 = joinPoints(footPoint4Off, kneePoint4Off, "z") 
        joint_2_3.name = "joint_2_3"

        #ui.messageBox("pause")

        #create extrusions for motor box
        kneeBoxWidth = 4.44
        kneeBoxLength = 4.44
        footBoxWidth = 4.44
        footBoxLength = 4.44

        #set material density
        plaDensity = 0.5

        


        
        # createMotorBox(body_1_2, legPoint3, motorWidth, motorLength, False)
        # createMotorBox(body_1_3, legPoint4, motorWidth, motorLength, False)
        # createMotorBox(body_1_0, legPoint1, motorWidth, motorLength, False)
        # createMotorBox(body_1_1, legPoint2, motorWidth, motorLength, False)

        # createMotorBox(base_link, bodyPoint1, motorWidth, motorLength, True)
        # createMotorBox(base_link, bodyPoint2, motorWidth, motorLength, True)
        # createMotorBox(base_link, bodyPoint3, motorWidth, motorLength, True)
        # createMotorBox(base_link, bodyPoint4, motorWidth, motorLength, True)

        # createKneeBox(body_1_0, kneePoint1, kneeBoxWidth, kneeBoxLength, True)
        # createKneeBox(body_1_1, kneePoint2, kneeBoxWidth, kneeBoxLength, True)
        # createKneeBox(body_1_2, kneePoint3, kneeBoxWidth, kneeBoxLength, True)
        # createKneeBox(body_1_3, kneePoint4, kneeBoxWidth, kneeBoxLength, True)

        # createKneeBox(body_2_0, footPoint1, footBoxWidth, footBoxLength, False)
        # createKneeBox(body_2_1, footPoint2, footBoxWidth, footBoxLength, False)
        # createKneeBox(body_2_2, footPoint3, footBoxWidth, footBoxLength, False)
        # createKneeBox(body_2_3, footPoint4, footBoxWidth, footBoxLength, False)


        # createElectronicsBox(base_link, 7.05, 12.85, -legPoint1.geometry.y)

        for occ in rootComp.occurrences:
            body = occ.component.bRepBodies[0]
            body.material = newMaterial

        # routeCable(base_link, bodyPoint1)
        # routeCable(base_link, bodyPoint2)
        # routeCable(base_link, bodyPoint3)
        # routeCable(base_link, bodyPoint4)

        doc = app.activeDocument

        doc.saveAs(doc.name + "copy", doc.dataFile.parentFolder, "", "")

        
        

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


#converts body to a component, so joints can be created and urdf can be exported
def create_component_from_body(existing_body):
    app = adsk.core.Application.get()
    design = app.activeProduct

    # Get the root component of the active design
    root_comp = design.rootComponent

    # Create a new component
    new_comp = root_comp.parentDesign.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component

    # Add the existing body to the new component
    new_comp.bRepBodies.add(existing_body)
    existing_body.deleteMe()

    return new_comp


def createMotorBox(comp: adsk.fusion.Component, point: adsk.fusion.SketchPoint, boxWidth, boxLength, isParent):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        rootComp = design.rootComponent
        sketches = comp.sketches
        planes = comp.constructionPlanes
        extrudes = comp.features.extrudeFeatures

        childExtDist = adsk.core.ValueInput.createByReal(1.8)
        parentExtDist = adsk.core.ValueInput.createByReal(1.8)
        childExtDistNeg = adsk.core.ValueInput.createByReal(-1.8)
        parentExtDistNeg = adsk.core.ValueInput.createByReal(-1.8)

        kneeExtDistNeg = adsk.core.ValueInput.createByReal(-3.05)
        kneeExtDist = adsk.core.ValueInput.createByReal(3.05)

        

        centerPoint = adsk.core.Point3D.create(point.geometry.x, point.geometry.z, -point.geometry.y)
        #ui.messageBox("ceternpoint: x = "+str(centerPoint.x)+ " y= "+str(centerPoint.y)+" z= "+str(centerPoint.z))
        cornerPoint = adsk.core.Point3D.create(centerPoint.x+boxWidth/2, centerPoint.y, centerPoint.z+ boxLength/2)
        #ui.messageBox("cornerpoint: x = "+str(cornerPoint.x)+ " y= "+str(cornerPoint.y)+" z= "+str(cornerPoint.z))

        if centerPoint.y > 0:
            posY = True
        else:
            posY = False

        plane = getPlane(point.geometry.z, comp.xZConstructionPlane, comp)

        sketch = sketches.add(plane)
        rectangles = sketch.sketchCurves.sketchLines 

        fillSketch = sketches.add(plane)
        fillRects = fillSketch.sketchCurves.sketchLines

        

        fillpoint1 = adsk.core.Point3D.create(centerPoint.x, -centerPoint.z, 0)
        fillpoint2 = adsk.core.Point3D.create(centerPoint.x+boxWidth/2+.2, -centerPoint.z + boxLength/2 +.2, 0)

        fillSketch.sketchPoints.add(fillpoint1)
        fillSketch.sketchPoints.add(fillpoint2)

        fillRect = fillRects.addCenterPointRectangle(fillpoint1, fillpoint2)
        fillprof = fillSketch.profiles[0]

        # if isParent:
        #     fillExt = extrudes.addSimple(fillprof, kneeParentExtDist, adsk.fusion.FeatureOperations.JoinFeatureOperation)
        # else:
        #     fillExt = extrudes.addSimple(fillprof, kneeChildExtDist, adsk.fusion.FeatureOperations.JoinFeatureOperation)

        if not isParent:
            if posY:
                fillExt = extrudes.addSimple(fillprof, kneeExtDist, adsk.fusion.FeatureOperations.JoinFeatureOperation)
            else:
                fillExt = extrudes.addSimple(fillprof, kneeExtDistNeg, adsk.fusion.FeatureOperations.JoinFeatureOperation)
        elif isParent:
            if posY:
                fillExt = extrudes.addSimple(fillprof, kneeExtDistNeg, adsk.fusion.FeatureOperations.JoinFeatureOperation)
            else:
                fillExt = extrudes.addSimple(fillprof, kneeExtDist, adsk.fusion.FeatureOperations.JoinFeatureOperation)




        centerPoint = adsk.core.Point3D.create(point.geometry.x, point.geometry.z, -point.geometry.y)
        #ui.messageBox("ceternpoint: x = "+str(centerPoint.x)+ " y= "+str(centerPoint.y)+" z= "+str(centerPoint.z))
        cornerPoint = adsk.core.Point3D.create(centerPoint.x+boxWidth/2, centerPoint.y, centerPoint.z+ boxLength/2)
        #ui.messageBox("cornerpoint: x = "+str(cornerPoint.x)+ " y= "+str(cornerPoint.y)+" z= "+str(cornerPoint.z))

        plane = getPlane(point.geometry.z, comp.xZConstructionPlane, comp)

        sketch = sketches.add(plane)
        rectangles = sketch.sketchCurves.sketchLines 

        
        #sketch.sketchPoints.add(point.geometry)
        

        point1 = adsk.core.Point3D.create(centerPoint.x, -centerPoint.z, 0)
        point2 = adsk.core.Point3D.create(centerPoint.x+boxWidth/2, -centerPoint.z + boxLength/2, 0)

        sketch.sketchPoints.add(point1)
        sketch.sketchPoints.add(point2)

        rectangle = rectangles.addCenterPointRectangle(point1, point2)
        prof = sketch.profiles[0]
        
        #extrude the sketch

        

        if not isParent:
            if posY:
                extrude1 = extrudes.addSimple(prof, childExtDist, adsk.fusion.FeatureOperations.CutFeatureOperation)
            else:
                extrude1 = extrudes.addSimple(prof, childExtDistNeg, adsk.fusion.FeatureOperations.CutFeatureOperation)
        elif isParent:
            if posY:
                extrude1 = extrudes.addSimple(prof, parentExtDistNeg, adsk.fusion.FeatureOperations.CutFeatureOperation)
            else:
                extrude1 = extrudes.addSimple(prof, parentExtDist, adsk.fusion.FeatureOperations.CutFeatureOperation)

        #add round peg for snap in:
        centerPoint = adsk.core.Point3D.create(point.geometry.x, point.geometry.z, -point.geometry.y)
        point1 = adsk.core.Point3D.create(centerPoint.x, -centerPoint.z, 0)

        if not isParent:
            if posY:
                pegplane = getPlane(point.geometry.z + 1.8, comp.xZConstructionPlane, comp)
            else:
                pegplane = getPlane(point.geometry.z - 1.8, comp.xZConstructionPlane, comp)
        elif isParent:
            if posY:
                pegplane = getPlane(point.geometry.z - 1.8, comp.xZConstructionPlane, comp)
            else:
                pegplane = getPlane(point.geometry.z + 1.8, comp.xZConstructionPlane, comp)

        pegDiam = 1.95

        sketch2 = sketches.add(pegplane)

        sketch2.sketchPoints.add(point1)
        circles = sketch2.sketchCurves.sketchCircles
        circle1 = circles.addByCenterRadius(point1, pegDiam/2 + 0.1)
        circle2 = circles.addByCenterRadius(point1, pegDiam/2)

        dist = adsk.core.ValueInput.createByReal(1.05)
        distNeg = adsk.core.ValueInput.createByReal(-1.05)

        fillDist = adsk.core.ValueInput.createByReal(1.25)
        fillDistNeg = adsk.core.ValueInput.createByReal(-1.25)

        if not isParent:
            if posY:
                #fillext = extrudes.addSimple(sketch2.profiles[0], fillDist, adsk.fusion.FeatureOperations.JoinFeatureOperation)
                extrude2 = extrudes.addSimple(sketch2.profiles[1], dist, adsk.fusion.FeatureOperations.CutFeatureOperation)
            else:
                #fillext = extrudes.addSimple(sketch2.profiles[0], fillDistNeg, adsk.fusion.FeatureOperations.JoinFeatureOperation)
                extrude2 = extrudes.addSimple(sketch2.profiles[1], distNeg, adsk.fusion.FeatureOperations.CutFeatureOperation)
        elif isParent:
            if posY:
                #fillext = extrudes.addSimple(sketch2.profiles[0], fillDistNeg, adsk.fusion.FeatureOperations.JoinFeatureOperation)
                extrude2 = extrudes.addSimple(sketch2.profiles[1], distNeg, adsk.fusion.FeatureOperations.CutFeatureOperation)
            else:
                #fillext = extrudes.addSimple(sketch2.profiles[0], fillDist, adsk.fusion.FeatureOperations.JoinFeatureOperation)
                extrude2 = extrudes.addSimple(sketch2.profiles[1], dist, adsk.fusion.FeatureOperations.CutFeatureOperation)
        
        
        
        return 

        


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def createKneeBox(comp: adsk.fusion.Component, point: adsk.fusion.SketchPoint, boxWidth, boxLength, isParent):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        rootComp = design.rootComponent
        sketches = comp.sketches
        planes = comp.constructionPlanes
        extrudes = comp.features.extrudeFeatures

        childExtDist = adsk.core.ValueInput.createByReal(1.8)
        parentExtDist = adsk.core.ValueInput.createByReal(1.8)
        childExtDistNeg = adsk.core.ValueInput.createByReal(-1.8)
        parentExtDistNeg = adsk.core.ValueInput.createByReal(-1.8)

        kneeChildExtDist = adsk.core.ValueInput.createByReal(-3.05)
        kneeParentExtDist = adsk.core.ValueInput.createByReal(3.05)

        centerPoint = adsk.core.Point3D.create(point.geometry.x, point.geometry.z, -point.geometry.y)
        #ui.messageBox("ceternpoint: x = "+str(centerPoint.x)+ " y= "+str(centerPoint.y)+" z= "+str(centerPoint.z))
        cornerPoint = adsk.core.Point3D.create(centerPoint.x+boxWidth/2, centerPoint.y, centerPoint.z+ boxLength/2)
        #ui.messageBox("cornerpoint: x = "+str(cornerPoint.x)+ " y= "+str(cornerPoint.y)+" z= "+str(cornerPoint.z))

        plane = getPlane(-point.geometry.y, comp.xYConstructionPlane, comp)

        sketch = sketches.add(plane)
        rectangles = sketch.sketchCurves.sketchLines 

        fillSketch = sketches.add(plane)
        fillRects = fillSketch.sketchCurves.sketchLines

        fillpoint1 = adsk.core.Point3D.create(centerPoint.x, centerPoint.y, 0)
        fillpoint2 = adsk.core.Point3D.create(centerPoint.x+boxWidth/2+.2, centerPoint.y + boxLength/2 +.2, 0)

        fillSketch.sketchPoints.add(fillpoint1)
        fillSketch.sketchPoints.add(fillpoint2)

        fillRect = fillRects.addCenterPointRectangle(fillpoint1, fillpoint2)
        fillprof = fillSketch.profiles[0]

        if isParent:
            fillExt = extrudes.addSimple(fillprof, kneeParentExtDist, adsk.fusion.FeatureOperations.JoinFeatureOperation)
        else:
            fillExt = extrudes.addSimple(fillprof, kneeChildExtDist, adsk.fusion.FeatureOperations.JoinFeatureOperation)

        # #sketch.sketchPoints.add(point.geometry)
        

        point1 = adsk.core.Point3D.create(centerPoint.x, centerPoint.y, 0)
        point2 = adsk.core.Point3D.create(centerPoint.x+boxWidth/2, centerPoint.y + boxLength/2, 0)

        sketch.sketchPoints.add(point1)
        sketch.sketchPoints.add(point2)

        rectangle = rectangles.addCenterPointRectangle(point1, point2)
        prof = sketch.profiles[0]
        
        #extrude the sketch

        if centerPoint.y > 0:
            posY = True
        else:
            posY = False

        if not isParent:
            extrude1 = extrudes.addSimple(prof, childExtDistNeg, adsk.fusion.FeatureOperations.CutFeatureOperation)
            
        elif isParent:
            
            extrude1 = extrudes.addSimple(prof, parentExtDist, adsk.fusion.FeatureOperations.CutFeatureOperation)
            
        #add bars for snap in:

        centerPoint = adsk.core.Point3D.create(point.geometry.x, point.geometry.z, -point.geometry.y)
        point1 = adsk.core.Point3D.create(centerPoint.x, centerPoint.y, 0)

        if not isParent:
            
            pegplane = getPlane(-point.geometry.y - 1.8, comp.xYConstructionPlane, comp)
            
        elif isParent:
            
            pegplane = getPlane(-point.geometry.y + 1.8, comp.xYConstructionPlane, comp)
            

        pegDiam = 1.95

        sketch2 = sketches.add(pegplane)

        sketch2.sketchPoints.add(point1)
        circles = sketch2.sketchCurves.sketchCircles
        #circle1 = circles.addByCenterRadius(point1, pegDiam/2 + 0.1)
        circle2 = circles.addByCenterRadius(point1, pegDiam/2)

        dist = adsk.core.ValueInput.createByReal(1.05)
        distNeg = adsk.core.ValueInput.createByReal(-1.05)

        fillDist = adsk.core.ValueInput.createByReal(1.25)
        fillDistNeg = adsk.core.ValueInput.createByReal(-1.25)

        if not isParent:
            
            extrude2 = extrudes.addSimple(sketch2.profiles[0], distNeg, adsk.fusion.FeatureOperations.CutFeatureOperation)
            
        elif isParent:
            
            extrude2 = extrudes.addSimple(sketch2.profiles[0], dist, adsk.fusion.FeatureOperations.CutFeatureOperation)
            
        
        
        return 

        


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def routeCable(comp: adsk.fusion.Component, point: adsk.fusion.SketchPoint):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        rootComp = design.rootComponent
        sketches = comp.sketches
        planes = comp.constructionPlanes
        
        extDepth = 3.5
        boxWidth = 2.6
        boxLength = 1.1

        if point.geometry.z > 0:
            posY = True
        else:
            posY = False

        if posY:
            plane = getPlane(point.geometry.z-extDepth, comp.xZConstructionPlane, comp)
        else:
            plane = getPlane(point.geometry.z+extDepth, comp.xZConstructionPlane, comp)

        sketch = sketches.add(plane)
        cablePoint = adsk.core.Point3D.create(point.geometry.x, point.geometry.y, 0)
        cornerPoint = adsk.core.Point3D.create(point.geometry.x+boxWidth/2, point.geometry.y+boxLength/2, 0)
        sketch.sketchPoints.add(cablePoint)
        sketch.sketchPoints.add(cornerPoint)

        rectangles = sketch.sketchCurves.sketchLines 

        rect = rectangles.addCenterPointRectangle(cablePoint, cornerPoint)

        prof = sketch.profiles[0]

        sweepPlane = getPlane(point.geometry.x, comp.yZConstructionPlane, comp)
        sweepHeight = comp.bRepBodies[0].boundingBox.maxPoint.z * 2

        sketch2 = sketches.add(sweepPlane)
        if posY:
            point1 = adsk.core.Point3D.create(point.geometry.y, point.geometry.z-extDepth, 0)
            point2 = adsk.core.Point3D.create(point.geometry.y, (point.geometry.z-extDepth)/2, 0)
            point3 = adsk.core.Point3D.create(-(point.geometry.y + sweepHeight), (point.geometry.z-extDepth)/2, 0)
        else:
            point1 = adsk.core.Point3D.create(point.geometry.y, point.geometry.z+extDepth, 0)
            point2 = adsk.core.Point3D.create(point.geometry.y, (point.geometry.z+extDepth)/2, 0)
            point3 = adsk.core.Point3D.create(-(point.geometry.y + sweepHeight), (point.geometry.z+extDepth)/2, 0)

        

        

        

        sketch2.sketchPoints.add(point1)
        sketch2.sketchPoints.add(point2)
        sketch2.sketchPoints.add(point3)

        lines = sketch2.sketchCurves.sketchLines
        line1 = lines.addByTwoPoints(point1, point2)
        line2 = lines.addByTwoPoints(point2, point3)

        if posY:
            arcRadius = abs((point.geometry.z-extDepth)/3)
        else:
            arcRadius = abs((point.geometry.z+extDepth)/3)
        
        # Add a fillet.
        arc = sketch2.sketchCurves.sketchArcs.addFillet(line1, line1.endSketchPoint.geometry, line2, line2.startSketchPoint.geometry, arcRadius)

        #add sweep
        sweeps = comp.features.sweepFeatures
        path = comp.features.createPath(line1)
        sweepInput = sweeps.createInput(prof, path, adsk.fusion.FeatureOperations.CutFeatureOperation)

        sweep = sweeps.add(sweepInput)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def createElectronicsBox(comp: adsk.fusion.Component, boxWidth, boxLength, planeHeight):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        rootComp = design.rootComponent
        sketches = comp.sketches
        planes = comp.constructionPlanes
        extrudes = comp.features.extrudeFeatures

        #xymax = getxymax()
        xymax = planeHeight

        #ui.messageBox("xymax = "+str(xymax))

        centerPoint = adsk.core.Point3D.create(0, 0, 0)
        cornerPoint = adsk.core.Point3D.create(boxLength/2, boxWidth/2, 0)
        plane = getPlane(xymax, comp.xYConstructionPlane, comp)
        sketch = sketches.add(plane)
        rectangles = sketch.sketchCurves.sketchLines 
        rect = rectangles.addCenterPointRectangle(centerPoint, cornerPoint)

        prof = sketch.profiles[0]

        extDist = adsk.core.ValueInput.createByReal(comp.bRepBodies[0].boundingBox.maxPoint.z)

        extrude1 = extrudes.addSimple(prof, extDist, adsk.fusion.FeatureOperations.CutFeatureOperation)


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def scaleFactorFromFaces(boxWidth, boxLength):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        rootComp = design.rootComponent

        minArea = -1

        #iterate through 8 faces of bodies, find minimum area
        for body in rootComp.bRepBodies:

            for face in body.faces:

                centroid = face.centroid
                val, normal = face.evaluator.getNormalAtPoint(centroid)

                if normal.x == 0 and normal.z == 0:
                     
                    if face.area < minArea or minArea == -1:
                        minArea = face.area

        #approximate face as a circle, fin its radius
        radius = math.sqrt(minArea/math.pi)

        #find diag of box
        diag = math.sqrt(boxLength**2 + boxWidth**2)

        scaleFactor = (diag/2)/radius
        
        return scaleFactor

        


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def scaleFactorFromBody(width):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        rootComp = design.rootComponent

        sketch = getIntersectSketch(rootComp.yZConstructionPlane, rootComp.bRepBodies[0])
        prof = sketch.profiles[0]

        areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        
        # Get area
        area = areaProps.area

        sketch.deleteMe()

        radius = math.sqrt(area/math.pi)

        return (width/2)/radius


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def createLegPoint(comp: adsk.fusion.Component):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        
        rootComp = design.rootComponent
        planes = comp.constructionPlanes
        sketches = comp.sketches

        comp.isActive = True

        if organic:

            for face in comp.bRepBodies[0].faces:
                #ui.messageBox("surface geo: "+ str(face.geometry.surfaceType))
                centroid = face.centroid
                val, normal = face.evaluator.getNormalAtPoint(centroid)

                #plane = getPlane(centroid.y, rootComp.xZConstructionPlane, comp)
                

                if normal.x == 0 and normal.z == 0:
                    #ui.messageBox("found correct normal")

                    #move body to origin
                    #transform = comp.transformOccurrences
                    #body = comp.bRepBodies[0]


                    sketch = sketches.add(comp.xZConstructionPlane)
                    point = adsk.core.Point3D.create(centroid.x, -centroid.z, centroid.y)
                    legPoint = sketch.sketchPoints.add(point)
                    break
                    
                

        elif prismatic:
            #todo: find face with largest area
            # ui.messageBox("havent done prismatic logic yet")
            # maxInx = 0
            # maxArea = 0
            # for i in range(len(comp.bRepBodies[0].faces)):
            #     if comp.bRepBodies[0].faces[i].area > maxArea:
            #         maxArea = comp.bRepBodies[0].faces[i].area
            #         maxInx = i

            # centroid = comp.bRepBodies[0].faces[maxInx].centroid
            for face in comp.bRepBodies[0].faces:
                #ui.messageBox("surface geo: "+ str(face.geometry.surfaceType))
                centroid = face.centroid
                val, normal = face.evaluator.getNormalAtPoint(centroid)

                #plane = getPlane(centroid.y, rootComp.xZConstructionPlane, comp)
                

                if normal.x == 0 and normal.z == 0:
                    #ui.messageBox("found correct normal")
                    sketch = sketches.add(comp.xZConstructionPlane)
                    point = adsk.core.Point3D.create(centroid.x, -centroid.z, centroid.y)
                    legPoint = sketch.sketchPoints.add(point)
                    break


        # plane = getPlane(centroid.y, comp.xZConstructionPlane, comp)
        # sketch = sketches.add(plane)

        # legPoint = sketch.sketchPoints.add(centroid)

        return legPoint
                
        #return comp.originConstructionPoint


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def createFootPoint(comp: adsk.fusion.Component):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        
        rootComp = design.rootComponent
        planes = comp.constructionPlanes
        sketches = comp.sketches

        comp.isActive = True


        for face in comp.bRepBodies[0].faces:
            #ui.messageBox("surface geo: "+ str(face.geometry.surfaceType))
            centroid = face.centroid
            val, normal = face.evaluator.getNormalAtPoint(centroid)

            #plane = getPlane(centroid.y, rootComp.xZConstructionPlane, comp)
            

            if normal.x == 0 and normal.y == 0:
                #ui.messageBox("found correct normal")

                #move body to origin
                #transform = comp.transformOccurrences
                #body = comp.bRepBodies[0]


                sketch = sketches.add(comp.xZConstructionPlane)
                point = adsk.core.Point3D.create(centroid.x, -centroid.z, centroid.y)
                legPoint = sketch.sketchPoints.add(point)
                break
                    
                

        return legPoint
                


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def offsetPoint(comp: adsk.fusion.Component, point: adsk.fusion.SketchPoint, offset, dir):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        
        rootComp = design.rootComponent
        planes = comp.constructionPlanes
        sketches = comp.sketches

        comp.isActive = True

        x = point.geometry.x
        y = point.geometry.y
        z = point.geometry.z

        if dir == "x":
            plane = getPlane(x+offset, comp.yZConstructionPlane, comp)
            offPoint = adsk.core.Point3D.create(y, z, 0)
        elif dir == "y":
            plane = getPlane(z+offset, comp.xZConstructionPlane, comp)
            offPoint = adsk.core.Point3D.create(x, y, 0)
        elif dir == "z":
            plane = getPlane(-y+offset, comp.xYConstructionPlane, comp)
            offPoint = adsk.core.Point3D.create(x, z, 0)
        
        sketch = sketches.add(plane)
        return sketch.sketchPoints.add(offPoint)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def createBodyPoint(comp, num):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        
        rootComp = design.rootComponent
        planes = comp.constructionPlanes
        sketches = comp.sketches

        comp.isActive = True

        if organic:
            
            for face in comp.bRepBodies[0].faces:
                #ui.messageBox("surface geo: "+ str(face.geometry.surfaceType))
                centroid = face.centroid
                val, normal = face.evaluator.getNormalAtPoint(centroid)

                
                

                if normal.x == 0 and normal.z == 0:
                    
                    sketch = sketches.add(comp.xZConstructionPlane)
                    point = adsk.core.Point3D.create(centroid.x, -centroid.z, centroid.y)
                    #ui.messageBox("found correct normal: num = "+str(num)+" x = "+str(point.x)+" y= "+str(point.y)+" z= "+str(point.z))
                    if point.x > 0 and point.z > 0:
                        if num == 1:
                            bodyPoint1 = sketch.sketchPoints.add(point)
                            #ui.messageBox("found bodyPoint1")
                        
                            return bodyPoint1
                    elif point.x < 0 and point.z > 0:
                        if num == 2:
                            bodyPoint2 = sketch.sketchPoints.add(point)
                            #ui.messageBox("found bodyPoint2")
                        
                            return bodyPoint2
                    elif point.x > 0 and point.z < 0:
                        if num == 3:
                            bodyPoint3 = sketch.sketchPoints.add(point)
                            #ui.messageBox("found bodyPoint3")
                        
                            return bodyPoint3
                    elif point.x < 0 and point.z < 0:
                        if num == 4:
                            bodyPoint4 = sketch.sketchPoints.add(point)
                            #ui.messageBox("found bodyPoint4")

                            return bodyPoint4
                    
                    
                

        elif prismatic:
            
            for face in comp.bRepBodies[0].faces:
                #ui.messageBox("surface geo: "+ str(face.geometry.surfaceType))
                centroid = face.centroid
                val, normal = face.evaluator.getNormalAtPoint(centroid)

                
                

                if normal.x == 0 and normal.z == 0:
                    
                    sketch = sketches.add(comp.xZConstructionPlane)
                    point = adsk.core.Point3D.create(centroid.x, -centroid.z, centroid.y)
                    
                    if point.x > 0 and point.y > 0:
                        if num == 1:
                            bodyPoint1 = sketch.sketchPoints.add(point)
                            
                        
                            return bodyPoint1
                    elif point.x < 0 and point.y > 0:
                        if num == 2:
                            bodyPoint2 = sketch.sketchPoints.add(point)
                            
                        
                            return bodyPoint2
                    elif point.x > 0 and point.y < 0:
                        if num == 3:
                            bodyPoint3 = sketch.sketchPoints.add(point)
                            
                        
                            return bodyPoint3
                    elif point.x < 0 and point.y < 0:
                        if num == 4:
                            bodyPoint4 = sketch.sketchPoints.add(point)
                            

                            return bodyPoint4


        


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def joinPoints(point1, point2, dir):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        
        rootComp = design.rootComponent

        geo1 = adsk.fusion.JointGeometry.createByPoint(point1)
        geo2 = adsk.fusion.JointGeometry.createByPoint(point2)
            
        #create one joint
        joints = rootComp.joints
        
        jointInput = joints.createInput(geo1, geo2)

        if dir == "x":
            jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.XAxisJointDirection)
        elif dir == "y":
            jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        elif dir == "z":
            jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)
        
        joint1 = joints.add(jointInput)
        
        return joint1


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def getxzmin():
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        rootComp = design.rootComponent

        crossarr = np.genfromtxt("C:/Users/ringe/Spring2024Research/cross_" + str(app.activeDocument.dataFile.name) + ".csv", delimiter=",")

        yzarr = crossarr[:, 0]
        xzarr = crossarr[:, 1]
        xyarr = crossarr[:, 2]
        inx = crossarr[:, 3]

        #find mins
        
        minima = argrelextrema(xzarr,np.less)
        xzminima = inx[minima]


        return np.sort(abs(xzminima))


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def getxymax():
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        rootComp = design.rootComponent

        crossarr = np.genfromtxt("C:/Users/ringe/Spring2024Research/cross_" + str(app.activeDocument.dataFile.name) + ".csv", delimiter=",")

        yzarr = crossarr[:, 0]
        xzarr = crossarr[:, 1]
        xyarr = crossarr[:, 2]
        inx = crossarr[:, 3]

        #find max
        xymax = np.argmax(xyarr)
        
        return inx[xymax]


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def sliceByMinima(mainbody, offsets, negate, target):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        rootComp = design.rootComponent
        planes = rootComp.constructionPlanes
        sketches = rootComp.sketches

        sliced = False
        cleanCut = False
        correctOffset = offsets[0]

        if negate:
            offsets = np.negative(offsets)

        for i in range(len(offsets)):
            testPlane = getPlane(offsets[i], rootComp.xZConstructionPlane, rootComp)

            sliceByPlane(mainbody, testPlane)

            #check number of valid bodies: if incorrect, combine everything back together, If correct, combine invalid bodies with closest valid body
            validBodies = adsk.core.ObjectCollection.create()
            invalidBodies = adsk.core.ObjectCollection.create()
            for body in rootComp.bRepBodies:
                if body.volume > thresholdVol:
                    validBodies.add(body)
                else:
                    invalidBodies.add(body)
            
            #validBodies should be 3 for a 4 legged robot
            if len(validBodies) == target:
                sliced = True
                correctOffset = offsets[i]
                if len(validBodies) == len(rootComp.bRepBodies):
                    cleanCut = True
                #fuse invalidBodies with validBodies
                
                #ui.messageBox("todo: fuse invalid with valid")
                fuseInvalid(validBodies, invalidBodies)
                #ui.messageBox("fuse finished")
                break

            else:
                #fuse everything back together
                combineAllBodies(mainbody)

            testPlane.deleteMe()

        return sliced, cleanCut, correctOffset


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def sliceByDX(mainbody, steps, negate, target):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        rootComp = design.rootComponent
        planes = rootComp.constructionPlanes
        sketches = rootComp.sketches

        sliced = False
        cleanCut = False
        correctOffset = 0

        #compute max y value plane can go to from bounding box
        bbox = mainbody.boundingBox
        upper_limit = bbox.maxPoint.y

        #when i = 100, i/denom = upperlimit
        denom = steps/upper_limit
        incr = 1

        if negate:
            steps = -steps
            incr = -incr

        for i in range(0, steps+1, incr):
            testPlane = getPlane(i/denom, rootComp.xZConstructionPlane, rootComp)

            sliceByPlane(mainbody, testPlane)

            #check number of valid bodies: if incorrect, combine everything back together, If correct, combine invalid bodies with closest valid body
            validBodies = adsk.core.ObjectCollection.create()
            invalidBodies = adsk.core.ObjectCollection.create()
            for body in rootComp.bRepBodies:
                if body.volume > thresholdVol:
                    validBodies.add(body)
                else:
                    invalidBodies.add(body)
            
            #validBodies should be 3 for a 4 legged robot
            if len(validBodies) == target:
                sliced = True
                correctOffset = i/denom
                if len(validBodies) == len(rootComp.bRepBodies):
                    cleanCut = True
                #fuse invalidBodies with validBodies
                #ui.messageBox("todo: fuse invalid with valid")
                #ui.messageBox("valid = "+str(len(validBodies))+" invalid = "+ str(len(invalidBodies)))
                fuseInvalid(validBodies, invalidBodies)
                #ui.messageBox("fuse finished")
                break
                

            else:
                #fuse everything back together
                combineAllBodies(mainbody)

            testPlane.deleteMe()

        return sliced, cleanCut, correctOffset


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def getPlane(offset, basePlane, comp):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        rootComp = design.rootComponent
        planes = comp.constructionPlanes

        planeInput = planes.createInput()
        offsetValue = adsk.core.ValueInput.createByReal(offset)
        planeInput.setByOffset(basePlane, offsetValue)
        planeOne = planes.add(planeInput)

        return planeOne


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def getIntersectSketch(plane, body):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        rootComp = design.rootComponent
        planes = rootComp.constructionPlanes
        sketches = rootComp.sketches

        sketch = sketches.add(plane)

        entities = []
        entities.append(body)
    

        sketch.intersectWithSketchPlane(entities)
        

        return sketch


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def sliceKnees(comp: adsk.fusion.Component):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        rootComp = design.rootComponent
        planes = rootComp.constructionPlanes

        kneeRangeUpper = rootComp.bRepBodies[0].boundingBox.minPoint.z
        for body in comp.bRepBodies:
            if body.boundingBox.minPoint.z > kneeRangeUpper:
                kneeRangeUpper = body.boundingBox.minPoint.z

        #kneeRangeUpper = rootComp.bRepBodies.itemByName("base_link").boundingBox.minPoint.z
        kneeRangeLower = (rootComp.bRepBodies.itemByName("body_1_0").boundingBox.minPoint.z + kneeRangeUpper)/3

        dz = abs(kneeRangeUpper-kneeRangeLower) / 10
        
        kneeHeight = findKneeHeight(kneeRangeUpper, kneeRangeLower, dz, comp)
        #kneeHeight = kneeHeight-4
        #ui.messageBox("kneeHeight = "+ str(kneeHeight))

        plane = getPlane(kneeHeight, comp.xYConstructionPlane, comp)

        sliceByPlane(comp.bRepBodies.itemByName("body_1_0"), plane)
        # determine which is foot
        for body in comp.bRepBodies:
            if body.name == "body_1_0":
                body1 = body
            elif body.name == "body_1_0 (1)":
                body2 = body
        if body1.boundingBox.minPoint.z > body2.boundingBox.minPoint.z:
            body1.name = "body_1_0"
            body2.name = "body_2_0"
        else:
            body1.name = "body_2_0"
            body2.name = "body_1_0"

        sliceByPlane(comp.bRepBodies.itemByName("body_1_1"), plane)
        # determine which is foot
        for body in comp.bRepBodies:
            if body.name == "body_1_1":
                body1 = body
            elif body.name == "body_1_1 (1)":
                body2 = body
        if body1.boundingBox.minPoint.z > body2.boundingBox.minPoint.z:
            body1.name = "body_1_1"
            body2.name = "body_2_1"
        else:
            body1.name = "body_2_1"
            body2.name = "body_1_1"

        sliceByPlane(comp.bRepBodies.itemByName("body_1_2"), plane)
        # determine which is foot
        for body in comp.bRepBodies:
            if body.name == "body_1_2":
                body1 = body
            elif body.name == "body_1_2 (1)":
                body2 = body
        if body1.boundingBox.minPoint.z > body2.boundingBox.minPoint.z:
            body1.name = "body_1_2"
            body2.name = "body_2_2"
        else:
            body1.name = "body_2_2"
            body2.name = "body_1_2"


        sliceByPlane(comp.bRepBodies.itemByName("body_1_3"), plane)
        # determine which is foot
        for body in comp.bRepBodies:
            if body.name == "body_1_3":
                body1 = body
            elif body.name == "body_1_3 (1)":
                body2 = body
        if body1.boundingBox.minPoint.z > body2.boundingBox.minPoint.z:
            body1.name = "body_1_3"
            body2.name = "body_2_3"
        else:
            body1.name = "body_2_3"
            body2.name = "body_1_3"

        return True

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def findKneeHeight(kneeRangeUpper, kneeRangeLower, dz, comp: adsk.fusion.Component):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        rootComp = design.rootComponent
        planes = rootComp.constructionPlanes
        sketches = rootComp.sketches

        body = rootComp.bRepBodies.itemByName("body_1_0")

        maxArea = 0
        maxHeight = kneeRangeLower
        i = kneeRangeLower

        while i <= kneeRangeUpper:
            plane = getPlane(i, rootComp.xYConstructionPlane, comp)
            sketch = getIntersectSketch(plane, body)
            area = 0
            for prof in sketch.profiles:
                areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
                area = area + areaProps.area

            if area > maxArea:
                maxArea = area
                maxHeight = i

            plane.deleteMe()
            sketch.deleteMe()

            i = i + dz

        return maxHeight

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


#return 2 bodies: body that was sliced, and new body created
def sliceByPlane(mainbody, prof):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        rootComp = design.rootComponent
        planes = rootComp.constructionPlanes
        sketches = rootComp.sketches

        #save list of previously existing bodies in rootComp
        tempBodies = rootComp.bRepBodies

        splitBodyFeats = rootComp.features.splitBodyFeatures
        splitBodyInput = splitBodyFeats.createInput(mainbody, prof, False)
        splitBodyFeats.add(splitBodyInput)

        #2 relevent bodies wil be mainbody still and body that is not in tempBodies list
        for body in rootComp.bRepBodies:
            match = False
            for tempBody in tempBodies:
                if body is tempBody:
                    match = True
            if match == False:
                newBody = body
                break

        #return mainbody and newBody
        if mainbody.volume > newBody.volume:
            body1 = mainbody
            body2 = newBody
        else:
            body2 = mainbody
            body1 = newBody
        

        return body1, body2


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def combineAllBodies(mainbody):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        rootComp = design.rootComponent
        planes = rootComp.constructionPlanes
        sketches = rootComp.sketches

        features = design.rootComponent.features

        combineFeatures = features.combineFeatures
        targetBody = mainbody
        toolBodies = adsk.core.ObjectCollection.create()

        for j in range(1, len(design.rootComponent.bRepBodies)):
            toolBodies.add(design.rootComponent.bRepBodies[j])
        
        combineInput = combineFeatures.createInput(targetBody, toolBodies)

        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation

        combineInput.isKeepToolBody = False

        combineFeature = combineFeatures.add(combineInput)

        design.rootComponent.bRepBodies[0].name = "base_link"
        



    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def fuseInvalid(validBodies, inValidBodies):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        rootComp = design.rootComponent
        planes = rootComp.constructionPlanes

        features = design.rootComponent.features

        for validBody in validBodies:
            validbbox = validBody.boundingBox
            toolBodies = adsk.core.ObjectCollection.create()
            targetBody = validBody
            #add invalid bodies to toolbodies
            for invalidBody in reversed(inValidBodies):
                invalidbbox = invalidBody.boundingBox

                if validbbox.intersects(invalidbbox):
                    #ui.messageBox(str(validBody.name)+" ttouches invalid "+str(invalidBody.name))
                    toolBodies.add(invalidBody)
                    inValidBodies.removeByItem(invalidBody)

            if len(toolBodies) == 0:
                continue

            # ui.messageBox("targetBody = "+str(targetBody.name))
            # ui.messageBox("tool bodies: ")
            # for body in toolBodies:
            #     ui.messageBox(body.name)

            combineFeatures = features.combineFeatures
            combineInput = combineFeatures.createInput(targetBody, toolBodies)
            combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
            combineInput.isKeepToolBody = False

            combineFeature = combineFeatures.add(combineInput)
            



    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))