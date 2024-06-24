import adsk.core, adsk.fusion, adsk.cam, traceback
import random
import math
#import SliceBody2

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        rootComp = design.rootComponent

        existingMaterial = app.favoriteMaterials.itemByName("My ABS Plastic")

        

        app.preferences.materialPreferences.defaultMaterial = existingMaterial

        

        # Set Level to Scale, ID on that level, and translation vector
        translation_in_meters = adsk.core.Vector3D.create(0.00, .02, -0.04)  # Adjust the Z-value as needed.
        
        translation = adsk.core.Vector3D.create(
            translation_in_meters.x * 100,
            translation_in_meters.y * 100,
            translation_in_meters.z * 100
        )

        target_level_to_scale = 0
        body_level = 1
        #body_id = random.randint(0, numLegs - 1)
        body_id = 3

        scaleWrapper(body_level, body_id, translation)

        


        # If not on the lowest level, fix the joint
        # if not joint_name == None:
        #     # Identify the bottom face of the bottom body
        #     face2, normal = findBottomFace(bottomBody)
        #     face1, normal = findTopFace(lower_occurrences_to_translate[0].bRepBodies[0])
        #     axis = adsk.fusion.JointDirections.YAxisJointDirection

        #     make_joint(face1, face2, axis, joint_name, rootComp)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def scaleWrapper(body_level, body_id, translation):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    rootComp = design.rootComponent

    existingMaterial = app.favoriteMaterials.itemByName("Polyethylene, Low Density")

    if app.favoriteMaterials.itemByName('My ABS Plastic') == None:

        newMaterial = design.materials.addByCopy(existingMaterial, 'My ABS Plastic')
        densityProp: adsk.core.FloatProperty = newMaterial.materialProperties.itemByName('Density')
        densityProp.value = 380.79

    else:
        newMaterial = app.favoriteMaterials.itemByName('My ABS Plastic')
        app.preferences.materialPreferences.defaultMaterial = newMaterial

    #ui.messageBox("in scale wrapper")

    # ASSUME THIS METADATA IS SOMEHOW IMPORTED
    numLegs = 4
    jointsPerLeg = 2

    affected_lower_limb_names = []
    for i in range(body_level + 1, jointsPerLeg + 1):
        limb_name = "body_" + str(i) + '_' + str(body_id)
        affected_lower_limb_names.append(limb_name)
        #ui.messageBox(limb_name)


    occurrence_to_scale = None

    # Get a reference to the limb that needs to be scaled and translated
    # And an array of references to lower extremities that just need to be translated
    lower_occurrences_to_translate = []
    joint_names_to_translate = []
    for occ in rootComp.occurrences:
        if occ.component.name == 'body_' + str(body_level) + '_' + str(body_id):
            #ui.messageBox('Selected Component to Scale: ' + occ.component.name)
            occurrence_to_scale = occ
        if occ.component.name in affected_lower_limb_names:
            lower_occurrences_to_translate.append(occ)
            joint_names_to_translate.append(occ.component.name.replace("body", "joint", 1)) # Assume lower joint names are same as the child bodies

    # Delete any joints to translate
    joint_name = None
    for j in design.rootComponent.joints:
        if j.name in joint_names_to_translate:
            joint_name = j.name

            revolute_joint = adsk.fusion.RevoluteJointMotion.cast(j.jointMotion)
            jointAxis = revolute_joint.rotationAxis
            
            
            j.deleteMe()

    #ui.messageBox("before split body in 2")

    topBody, bottomBody = split_body_int_two(occurrence_to_scale, rootComp)

    #ui.messageBox("after splitbody in two")

    bottom_face_of_top_body, normal = findBottomFace(topBody)
    top_face_of_bottom_body, normal = findTopFace(bottomBody)

    #ui.messageBox("checkpoint 1")

    # Translate body and all lower extremities
    translate_body(bottomBody, rootComp, translation)
    for occ in lower_occurrences_to_translate:
        translate_body(occ.bRepBodies[0], rootComp, translation)

    #ui.messageBox("checkpoint 2")

    # Create the loft feature
    loftFeats = rootComp.features.loftFeatures
    loftInput = loftFeats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    # Use the faces directly if possible in your API version
    loftInput.loftSections.add(top_face_of_bottom_body)
    loftInput.loftSections.add(bottom_face_of_top_body)

    #loftInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation

    #ui.messageBox("checkpoint 3")

    # Create the loft
    loftFeature = loftFeats.add(loftInput)

    

    toolBodies = adsk.core.ObjectCollection.create()
    rootComp.bRepBodies[0].material = newMaterial
    toolBodies.add(rootComp.bRepBodies[0])
    toolBodies.add(topBody)
    # Combine the loft body with the bottom body (target body)
    #ui.messageBox("before combine")
    combineFeature = combineBodies(rootComp, bottomBody, toolBodies)
    
    #ui.messageBox("checkpoint 4")

    # remake_joints_if_needed
    if body_level == 1:
        component_one = occurrence_to_scale.component
        component_two = lower_occurrences_to_translate[0].component

        newKneePoint = createFootPoint(component_one)
        newFootPoint = createFootPoint(component_two)

        kneePointOff = offsetPoint(component_one, newKneePoint, -2, "z")
        footPointOff = offsetPoint(component_two, newFootPoint, 2, "z")

        #ui.messageBox("before joinpoints")

        newJoint = joinPoints(footPointOff, kneePointOff, jointAxis)   
        newJoint.name = "joint_2_"+str(body_id)

        #ui.messageBox("after joinpoints")

        newKneePoint.deleteMe()
        newFootPoint.deleteMe()





def make_joint(face1, face2, axis, joint_name, rootComp):
    geo1 = adsk.fusion.JointGeometry.createByPlanarFace(face1, None, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    geo2 = adsk.fusion.JointGeometry.createByPlanarFace(face2, None, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)

    # Create joint input
    joints = rootComp.joints
    jointInput = joints.createInput(geo1, geo2)
    
    # Axis should not be hardcoded
    # Define the joint type (e.g., revolute)
    jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)

    kneeOffset = adsk.core.ValueInput.createByReal(2)
    jointInput.offset = kneeOffset
    jointInput.isFlipped = True
    
    # Create and rename the joint
    joint = joints.add(jointInput)
    joint.name = joint_name

def highlight_face_by_sketch(face):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = design.rootComponent

    # Create a new sketch on the selected face
    sketches = rootComp.sketches
    sketch = sketches.add(face)

    return sketch

def combineBodies(rootComp, targetBody, toolBodies):
    # Create combine feature input
    combineFeatures = rootComp.features.combineFeatures
    combineInput = combineFeatures.createInput(targetBody, toolBodies)
    
    # Set the operation to join
    combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    
    # Create the combine feature
    combineFeature = combineFeatures.add(combineInput)
    return combineFeature

def findTopFace(body):
    maxZ = None
    topFace = None
    for face in body.faces:
        # Get the centroid directly from the BRepFace properties
        centroid = face.centroid
        
        # Get the evaluator from the face
        evaluator = face.evaluator

        # Get the parameters at the centroid of the face
        result, param = evaluator.getParameterAtPoint(centroid)
        if not result:
            continue  # Skip if cannot get parameter at centroid

        # Evaluate the normal at these parameters
        result, normal = evaluator.getNormalAtParameter(param)
        if not result:
            continue  # Skip if cannot evaluate normal

        # Check if the normal points upwards
        if normal.z > 0.99:
            # Use the Z-coordinate of the centroid for comparison
            if maxZ is None or centroid.z > maxZ:
                maxZ = centroid.z
                topFace = face
    return topFace, normal

def findBottomFace(body):
    minZ = None
    bottomFace = None
    for face in body.faces:
        # Get the centroid directly from the BRepFace properties
        centroid = face.centroid
        
        # Get the evaluator from the face
        evaluator = face.evaluator

        # Get the parameters at the centroid of the face
        result, param = evaluator.getParameterAtPoint(centroid)
        if not result:
            continue  # Skip if cannot get parameter at centroid

        # Evaluate the normal at these parameters
        result, normal = evaluator.getNormalAtParameter(param)
        if not result:
            continue  # Skip if cannot evaluate normal

        # Check if the normal points downwards
        if normal.z < -0.99:
            # Use the Z-coordinate of the centroid for comparison
            if minZ is None or centroid.z < minZ:
                minZ = centroid.z
                bottomFace = face
    return bottomFace, normal


def translate_body(body, root, translation_vector):
    entities = adsk.core.ObjectCollection.create()
    entities.add(body)

    # Create the translation matrix for moving the body.
    matrix = adsk.core.Matrix3D.create()
    matrix.translation = translation_vector

    # Create a move feature input.
    moveFeatures = root.features.moveFeatures
    moveInput = moveFeatures.createInput(entities, matrix)

    # Perform the move operation.
    moveFeature = moveFeatures.add(moveInput)
    return

def split_body_int_two(occurrence, root):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    # Get the bounding box of the body
    boundingBox = occurrence.boundingBox

    # Calculate the Z midpoint of the bounding box
    zMid = (boundingBox.minPoint.z + boundingBox.maxPoint.z) / 2
    #zMid = zMid+.2
    

    # Create a construction plane at the midpoint Z value
    # First, create a point at the desired location for the plane
    planes = root.constructionPlanes
    planeInput = planes.createInput()

    # Set the construction plane to be offset from the XY plane of the root component
    xyPlane = root.xYConstructionPlane
    offsetValue = adsk.core.ValueInput.createByReal(zMid)
    planeInput.setByOffset(xyPlane, offsetValue)

    # Create the construction plane
    constPlane = planes.add(planeInput)

    

    bodyToSplit = occurrence.bRepBodies[0]

    crossSketch = getIntersectSketch(constPlane, bodyToSplit)

    # if len(crossSketch.profiles) > 1:
    #     planeInput = planes.createInput()
    #     #ui.messageBox("bad zmid")
    #     # Set the construction plane to be offset from the XY plane of the root component
    #     xyPlane = root.xYConstructionPlane
    #     offsetValue = adsk.core.ValueInput.createByReal((zMid - boundingBox.minPoint.z)/2)
    #     planeInput.setByOffset(xyPlane, offsetValue)

    #     # Create the construction plane
    #     constPlane = planes.add(planeInput)

    crossSketch.deleteMe()

    # Assuming the last created construction plane is the splitting tool.
    splittingTool = root.constructionPlanes.item(root.constructionPlanes.count - 1)
    #splittingTool = root.constructionPlanes.item(constPlane)

    # Get the SplitBodyFeatures collection.
    splitBodyFeatures = root.features.splitBodyFeatures

    # Create the SplitBodyFeatureInput.
    extendSplittingPlane = True  # Optionally extend the splitting tool if necessary.
    splitBodyInput = splitBodyFeatures.createInput(bodyToSplit, splittingTool, extendSplittingPlane)
    
    # Create the split body feature.
    splitBodyFeature = splitBodyFeatures.add(splitBodyInput)

    constPlane.deleteMe()

    # Initialize variables to store the top and bottom bodies and their Z positions.
    topBody = None
    bottomBody = None
    topBodyZ = float('-inf')
    bottomBodyZ = float('inf')

    #ui.messageBox("bodies: "+ str(len(occurrence.bRepBodies)))

    if occurrence.bRepBodies[0].boundingBox.minPoint.z > occurrence.bRepBodies[1].boundingBox.minPoint.z:
        topBody = occurrence.bRepBodies[0]
        bottomBody = occurrence.bRepBodies[1]
    else:
        topBody = occurrence.bRepBodies[1]
        bottomBody = occurrence.bRepBodies[0]

    topBody.name = 'top'
    bottomBody.name = 'bottom'

    return topBody, bottomBody


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



def joinPoints(point1, point2, jointAxis):
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

        # if dir == "x":
        #     jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.XAxisJointDirection)
        # elif dir == "y":
        #     jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        # elif dir == "z":
        #     jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)

        jointInput.setAsRevoluteJointMotion(jointAxis)
        
        joint1 = joints.add(jointInput)
        
        return joint1


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