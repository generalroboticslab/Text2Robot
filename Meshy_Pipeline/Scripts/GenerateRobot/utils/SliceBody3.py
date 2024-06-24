#Author-
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback
import math
import sys

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
        sketches = rootComp.sketches

        if len(rootComp.bRepBodies) > 1:
            raise Exception("ERROR: Too many Bodies")

        design.rootComponent.bRepBodies[0].name = "base_link"
        mainbody = design.rootComponent.bRepBodies.itemByName("base_link")

        initialVolume = mainbody.volume
        #hard coded targetVolume in cm^3
        targetVolume = 6300

        #ui.messageBox("initial: "+str(initialVolume)+" target: "+str(targetVolume))

        scaleNum = math.cbrt(targetVolume/initialVolume)
        scaleFactor = adsk.core.ValueInput.createByReal(scaleNum)

        inputColl = adsk.core.ObjectCollection.create()

        #add body
        inputColl.add(mainbody)

        scaleInput = scales.createInput(inputColl, rootComp.originConstructionPoint, scaleFactor)
        
        #Scale body to targetVolume
        scale = scales.add(scaleInput)


        global thresholdVol
        thresholdVol = 0.03 * mainbody.volume 

        global thresholdArea
        thresholdArea = 0.3 * math.cbrt(mainbody.volume)
        #ui.messageBox("threshArea: "+str(thresholdArea))

        motorWidth = 3.35
        motorLength = 5.35

        

        #plot centroids of all substantial slices to create rough skeleton
        sketch3D = plotCentroids(mainbody, rootComp)
        

        armpits = findArmpits(mainbody, rootComp, sketch3D)

        ui.messageBox("pits found: "+str(len(armpits)))
        number_of_legs = len(armpits)

        for pit in armpits:
            sliceProf = sliceLeg(mainbody, rootComp, pit, armpits, sketch3D)
            if sliceProf is None:
                #ui.messageBox("ERROR: ensue default slice")
                defaultProf = getDefaultProf(mainbody, rootComp, pit)
                sliceByProf(mainbody, defaultProf, pit)

                fuseUnderBelly(rootComp)

                retryFuse(rootComp)
            else:
                sliceByProf(mainbody, sliceProf, pit)
                # defaultProf = getDefaultProf(mainbody, rootComp, pit)
                # sliceByProf(mainbody, defaultProf, pit)

                # fuseUnderBelly(rootComp)

                retryFuse(rootComp)

            mainbody = findMainBody(rootComp)

        mainbody = findMainBody(rootComp)
        retryFuse(rootComp)

        i = 0

        while len(rootComp.bRepBodies) > number_of_legs+1:
            #fuse the smallest body 
            ui.messageBox("calling fuseSmallest:")
            fuseSmallest(rootComp)
            if i > 8:
                ui.messageBox("error: infinite loop avoided")
                break
            i = i+1
            

        for sketch in rootComp.sketches:
            sketch.isVisible = False
            
        
        
        legBodies = adsk.core.ObjectCollection.create()
        leg_comps = adsk.core.ObjectCollection.create()
        mainbbox = mainbody.boundingBox
        
        for body in rootComp.bRepBodies:
            if body == mainbody:
                continue

            bbox = body.boundingBox

            if mainbbox.intersects(bbox):
                legBodies.add(body)


        for i in range(len(legBodies)):
            leg = legBodies[i]

            leg.name = "leg_"+str(i)

            legComp: adsk.fusion.Component = create_component_from_body(rootComp.bRepBodies.itemByName("leg_"+str(i)))
            legComp.name = "leg_"+str(i)

            leg_comps.add(legComp)


        base_link = create_component_from_body(mainbody)
        base_link.name = "base_link"

        joints_per_leg = 2

        foot_comps = adsk.core.ObjectCollection.create()

        for leg in leg_comps:
            joinLegToBody(leg, base_link, 1)

        for leg in leg_comps:
            newBod = sliceKnee(leg, joints_per_leg-1)
            
            newFoot = create_component_from_body(newBod)

            newFoot.name = "foot_" + leg.name[-1]
            foot_comps.add(newFoot)

            joinLegToBody(newFoot, leg, 2)



        kneeBoxWidth = 3.35
        kneeBoxLength = 4.3
        footBoxWidth = 2.35
        footBoxLength = 4.65
        motorWidth = 3.35
        motorLength = 5.35
        motorDepthChild = 2
        motorDepthParent = 3.7

        # faces = getFlatFaces(base_link.bRepBodies[0])
        # i = 0
        # for face in faces:
        #     createMotorBox(base_link, face, motorWidth, motorLength, motorDepthChild)
        #     i = i+1
        #     if i >= number_of_legs:
        #         break
            


        # for foot in foot_comps:
            
        #     faces = getFlatFaces(foot.bRepBodies[0])
        #     for face in faces:
        #         createMotorBox(foot, face, motorWidth, motorLength, motorDepthChild)


        

        # for leg in leg_comps:
            
        #     faces = getFlatFaces(leg.bRepBodies[0])
        #     for face in faces:
        #         createMotorBox(leg, face, motorWidth, motorLength, motorDepthChild)
                

        
        createElectronicsBox(base_link, 7.05, 12.85)

        

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def sliceKnee(leg: adsk.fusion.Component, numCuts):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    sketches = leg.sketches
    sketch = sketches.add(leg.xYConstructionPlane)

    body = leg.bRepBodies[0]
    bbox = body.boundingBox

    totalHeight = bbox.maxPoint.z - bbox.minPoint.z

    lowerBound = math.ceil(bbox.minPoint.z + totalHeight/3)
    upperBound = math.floor(bbox.maxPoint.z - totalHeight/4)

    maxArea = 0
    
    maxHeight = None

    for i in range(lowerBound, upperBound):
        plane = getPlane(i, leg.xYConstructionPlane, leg)
        sketch = getIntersectSketch(plane, body, leg)
        area = 0
        for prof in sketch.profiles:
            areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
            area = areaProps.area

            if area > maxArea:
                maxArea = area
                
                maxHeight = i

        sketch.deleteMe()
        plane.deleteMe()

    # areaProps = maxProf.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
    # centroid = areaProps.centroid

    plane = getPlane(maxHeight, leg.xYConstructionPlane, leg)

    splitBodyFeats = leg.features.splitBodyFeatures
    splitBodyInput = splitBodyFeats.createInput(body, plane, True)
    splitBodyFeats.add(splitBodyInput)

    i = 0
    while len(leg.bRepBodies) > 2:
        ui.messageBox("calling fuseSmallest")
        fuseSmallest(leg)
        if i > 8:
            ui.messageBox("error: infinite loop avoided")
            break
        i=i+1


    minHeight = sys.maxsize
    minBody = None

    for body in leg.bRepBodies:
        bbox = body.boundingBox

        if bbox.minPoint.z < minHeight:
            minHeight = bbox.minPoint.z
            minBody = body

    return minBody


def joinLegToBody(leg: adsk.fusion.Component, base_link: adsk.fusion.Component, dirNum):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    legSketches = leg.sketches
    bodySketches = base_link.sketches
    axes = base_link.constructionAxes

    legFaces = getFlatFaces(leg.bRepBodies[0])
    base_link_faces = getFlatFaces(base_link.bRepBodies[0])

    legFace: adsk.fusion.BRepFace = legFaces[0]

    #find closest baseLink face to legFace
    minDist = sys.maxsize
    minFace = None
    
    legFaceCentroid = legFace.centroid

    for face in base_link_faces:
        centroid = face.centroid
        dist = math.sqrt((centroid.x-legFaceCentroid.x)**2+(centroid.y-legFaceCentroid.y)**2+(centroid.z-legFaceCentroid.z)**2)

        if dist < minDist:
            minDist = dist
            minFace = face

    legSketch = legSketches.add(leg.xYConstructionPlane)
    bodySketch = bodySketches.add(base_link.xYConstructionPlane)

    legPoint = legSketch.sketchPoints.add(legFaceCentroid)
    bodyPoint = bodySketch.sketchPoints.add(minFace.centroid)

    #Get axis normal to legFace
    axisInput = axes.createInput()

    normal = legFace.evaluator.getNormalAtPoint(legFace.centroid)

    # constructionPointInput = rootComp.constructionPoints.createInput()
    # constructionPointInput.setByPoint(legFace.centroid)
    # construction_point = rootComp.constructionPoints.add(constructionPointInput)

    # axisInput.setByNormalToFaceAtPoint(legFace, legPoint)

    point = legPoint.worldGeometry

    # normalAxis = axes.add(axisInput)
    translated_point = adsk.core.Point3D.create(point.x + normal[1].x, point.y + normal[1].y, point.z + normal[1].z)

    sketch = base_link.sketches.add(rootComp.xYConstructionPlane)
    #line = sketch.sketchCurves.sketchLines.addByTwoPoints(legFaceCentroid, translated_point)
    
    axispoint1 = sketch.sketchPoints.add(point)
    axispoint2 = sketch.sketchPoints.add(translated_point)
    axisInput.setByTwoPoints(axispoint1, axispoint2)

    
    

    normalAxis = axes.add(axisInput)

    

            
    joinPoints(leg, base_link, normalAxis, legFace, minFace, dirNum)


def joinPoints(leg, base_link, axis, face1, face2, dirNum):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        
        rootComp = design.rootComponent

        # geo1 = adsk.fusion.JointGeometry.createByPoint(point1)
        # geo2 = adsk.fusion.JointGeometry.createByPoint(point2)

        geoface1 = adsk.fusion.JointGeometry.createByPlanarFace(face1, None, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
        geoface2 = adsk.fusion.JointGeometry.createByPlanarFace(face2, None, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
            

        shoulderOffset = adsk.core.ValueInput.createByReal(1)
        kneeOffset = adsk.core.ValueInput.createByReal(2)

        #create one joint
        joints = rootComp.joints
        
        jointInput = joints.createInput(geoface1, geoface2)

        
        
        # jointInput.jointMotion = adsk.fusion.RevoluteJointMotion
        if dirNum == 1:
            # Get axis normal to legFace
            jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection, axis)
            jointInput.offset = shoulderOffset

            


        elif dirNum == 2:
            jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.YAxisJointDirection, axis)
            jointInput.offset = kneeOffset
        elif dirNum == 3:
            jointInput.setAsRevoluteJointMotion(adsk.fusion.JointDirections.XAxisJointDirection, axis)
            jointInput.offset = kneeOffset
        else:
            ui.messageBox("error: improper dirNum")

        

        
        jointInput.isFlipped = True
        
        
        joint1 = joints.add(jointInput)

        
        
        return joint1


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))



def createMotorBox(comp: adsk.fusion.Component, face: adsk.fusion.BRepFace, boxWidth, boxLength, boxDepth):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        rootComp = design.rootComponent
        sketches = comp.sketches
        planes = comp.constructionPlanes
        extrudes = comp.features.extrudeFeatures

        fillExtDist = adsk.core.ValueInput.createByReal(-(boxDepth+0.2))
        cutExtDist = adsk.core.ValueInput.createByReal(-(boxDepth))

        profsketch = sketches.add(face)

        prof = profsketch.profiles[0]

        areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        centerPoint = areaProps.centroid
        

        #centerPoint = face.centroid
        plane = getPlane(0, face, comp)
        sketch = sketches.add(plane)

        point1 = sketch.sketchPoints.add(centerPoint)

        cornerPoint = adsk.core.Point3D.create(centerPoint.x + boxLength/2 + 0.2, centerPoint.y + boxWidth/2 + 0.2, 0)

        point2 = sketch.sketchPoints.add(cornerPoint)

        rectangles = sketch.sketchCurves.sketchLines

        rectangle = rectangles.addCenterPointRectangle(centerPoint, point2)
        prof = sketch.profiles[0]

        extrude1 = extrudes.addSimple(prof, fillExtDist, adsk.fusion.FeatureOperations.JoinFeatureOperation)

        cutsketch = sketches.add(plane)
        cornerPoint = adsk.core.Point3D.create(centerPoint.x + boxLength/2, centerPoint.y + boxWidth/2, 0)

        point2 = cutsketch.sketchPoints.add(cornerPoint)

        rectangles = cutsketch.sketchCurves.sketchLines

        rectangle = rectangles.addCenterPointRectangle(centerPoint, point2)
        prof = cutsketch.profiles[0]

        extrude2 = extrudes.addSimple(prof, cutExtDist, adsk.fusion.FeatureOperations.CutFeatureOperation)
        
        barDiam = 0.3
        legDepth = 2
        baseDepth = 3.7
                
        # plane2 = getPlane(point1.x - boxWidth/2, comp.yZConstructionPlane, comp)
        # if not isParent:
        #     if posY:
        #         #ui.messageBox("posY hit")
        #         barPoint1 = adsk.core.Point3D.create(point1.y + 1.4, centerPoint.y+legDepth-barDiam/2, 0)
        #         barPoint2 = adsk.core.Point3D.create(point1.y - 1.4, centerPoint.y+legDepth-barDiam/2, 0)
        #     else:
        #         barPoint1 = adsk.core.Point3D.create(point1.y + 1.4, centerPoint.y-legDepth+barDiam/2, 0)
        #         barPoint2 = adsk.core.Point3D.create(point1.y - 1.4, centerPoint.y-legDepth+barDiam/2, 0)
        # elif isParent:
        #     if posY:
        #         barPoint1 = adsk.core.Point3D.create(point1.y + 1.4, centerPoint.y-baseDepth+barDiam/2, 0)
        #         barPoint2 = adsk.core.Point3D.create(point1.y - 1.4, centerPoint.y-baseDepth+barDiam/2, 0)
        #     else:
        #         barPoint1 = adsk.core.Point3D.create(point1.y + 1.4, centerPoint.y+baseDepth-barDiam/2, 0)
        #         barPoint2 = adsk.core.Point3D.create(point1.y - 1.4, centerPoint.y+baseDepth-barDiam/2, 0)
        
        # sketch2 = sketches.add(plane2)
        # sketch2.sketchPoints.add(barPoint1)
        # sketch2.sketchPoints.add(barPoint2)

        # circles = sketch2.sketchCurves.sketchCircles
        # circle1 = circles.addByCenterRadius(barPoint1, barDiam/2)
        # circle2 = circles.addByCenterRadius(barPoint2, barDiam/2)

        # coll = adsk.core.ObjectCollection.create()
        # coll.add(sketch2.profiles[0])
        # coll.add(sketch2.profiles[1])

        # dist = adsk.core.ValueInput.createByReal(boxWidth)
        # extrude2 = extrudes.addSimple(coll, dist, adsk.fusion.FeatureOperations.JoinFeatureOperation)
        
        return 

        


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def createElectronicsBox(comp: adsk.fusion.Component, boxWidth, boxLength):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        rootComp = design.rootComponent
        sketches = comp.sketches
        planes = comp.constructionPlanes
        extrudes = comp.features.extrudeFeatures

        body = comp.bRepBodies[0]
        bbox = body.boundingBox

        lowerBound = math.ceil(bbox.minPoint.z)
        upperBound = math.floor(bbox.maxPoint.z)

        maxArea = 0
        
        maxHeight = None

        for i in range(lowerBound, upperBound):
            plane = getPlane(i, comp.xYConstructionPlane, comp)
            sketch = getIntersectSketch(plane, body, comp)
            area = 0
            for prof in sketch.profiles:
                areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
                area = areaProps.area

                if area > maxArea:
                    maxArea = area
                    
                    maxHeight = i

            sketch.deleteMe()
            plane.deleteMe()

        #xymax = getxymax()
        xymax = maxHeight

        #ui.messageBox("xymax = "+str(xymax))

        sketch = sketches.add(comp.xYConstructionPlane)

        centerPoint = adsk.core.Point3D.create(0, 0, maxHeight)
        cornerPoint = adsk.core.Point3D.create(boxLength/2, boxWidth/2, maxHeight)
        #plane = getPlane(xymax, comp.xYConstructionPlane, comp)
        
        rectangles = sketch.sketchCurves.sketchLines 
        rect = rectangles.addCenterPointRectangle(centerPoint, cornerPoint)

        prof = sketch.profiles[0]

        extDist = adsk.core.ValueInput.createByReal(comp.bRepBodies[0].boundingBox.maxPoint.z - comp.bRepBodies[0].boundingBox.minPoint.z)

        extrude1 = extrudes.addSimple(prof, extDist, adsk.fusion.FeatureOperations.CutFeatureOperation)


    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def getDefaultProf(body: adsk.fusion.BRepBody, comp:adsk.fusion.Component, pit: adsk.fusion.SketchPoint):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    planes = comp.constructionPlanes
    sketches = rootComp.sketches

    plane = getPlane(pit.worldGeometry.z, comp.xYConstructionPlane, comp)
    sketch = getIntersectSketch(plane, body)

    minDist = sys.maxsize
    minProf = None

    for prof in sketch.profiles:
        areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        centroid = areaProps.centroid
        dist = math.sqrt((centroid.x-pit.worldGeometry.x)**2 + (centroid.y-pit.worldGeometry.y)**2)

        if dist < minDist:
            minDist = dist
            minProf = prof

    return minProf



def findMainBody(comp: adsk.fusion.Component):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)

    for body in comp.bRepBodies:
        bbox = body.boundingBox

        xmax = bbox.maxPoint.x
        xmin = bbox.minPoint.x
        ymax = bbox.maxPoint.y
        ymin = bbox.minPoint.y

        if (xmin < 0 < xmax and
            ymin < 0 < ymax) and body.volume > 10*thresholdVol:
            return body
        
    ui.messageBox("ERROR: no mainbody found")

# Given a pit point, return an optimal profile to slice the leg containing pit
def sliceLeg(body: adsk.fusion.BRepBody, comp: adsk.fusion.Component, pit: adsk.fusion.SketchPoint, armpits: adsk.core.ObjectCollection, sketch3D: adsk.fusion.Sketch):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    planes = comp.constructionPlanes
    sketches = rootComp.sketches

    #potentialPlanes = adsk.core.ObjectCollection.create()
    #dict storing plane as key, cross section area as value 
    #ui.messageBox("sliceLeg called")
    
    validProfs = adsk.core.ObjectCollection.create()
    zeroValue = adsk.core.ValueInput.createByReal(0)

    #sketch = sketches.add(comp.xYConstructionPlane)
    sketchLines = sketch3D.sketchCurves.sketchLines

    for otherPit in armpits:


        if pit != otherPit:
            #create line between 2 pits, create plane normal to line and through pit
            pitLine = sketchLines.addByTwoPoints(pit, otherPit)

            sliceProf = sliceAlongPath(body, comp, pit, pitLine)
            if sliceProf != None:
                
                validProfs.add(sliceProf)


    minArea = sys.maxsize
    minProf = None
    for prof in validProfs:
        

        areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        area = areaProps.area

        if area < minArea:
            minArea = area
            minProf = prof

    # if minProf is None:
    #     ui.messageBox("minprof returned by sliceLeg is none")
    return minProf 

    # Test each potentialPlane, if creates valid slice , add to valid planes
    
# Given path starting at pit, return an optimal profile to slice leg along this path
def sliceAlongPath(mainbody: adsk.fusion.BRepBody, comp: adsk.fusion.Component, pit: adsk.fusion.SketchPoint, path: adsk.fusion.SketchLine):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    planes = comp.constructionPlanes
    sketches = rootComp.sketches

    #ui.messageBox("slice along path called")

    #create reference x and y points for pit
    # zeroValue = adsk.core.ValueInput.createByReal(0)

    # plane_input = planes.createInput()
    # plane_input.setByDistanceOnPath(path, zeroValue)
    # plane = planes.add(plane_input)

    # # slice along plane
    # sketch = getIntersectSketch(plane, mainbody)

    # #find prof containing pit
    # minDist = sys.maxsize
    # zeroX = 0
    # zeroY = 0
    # for prof in sketch.profiles:
    #     areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
    #     centroid = areaProps.centroid
    #     dist = math.sqrt((centroid.x-pit.geometry.x)**2+(centroid.y-pit.geometry.z))
    #     if dist < minDist:
    #         minDist=dist
    #         zeroX = centroid.x
    #         zeroY = centroid.y

    profsInRange = adsk.core.ObjectCollection.create()
    bestProfOnPath = None
    minCost = sys.maxsize

    for i in range(8):
        iValue = adsk.core.ValueInput.createByReal(0.05*i)

        plane_input = planes.createInput()
        plane_input.setByDistanceOnPath(path, iValue)
        plane = planes.add(plane_input)

        #ui.messageBox(str(0.05*i)+" cm along path")

        # slice along plane
        sketch = getIntersectSketch(plane, mainbody)



        #find prof within range of pit, with min area

        #find area of prof closest to pit
        
        for prof in sketch.profiles:
            areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
            centroid = areaProps.centroid

            if areaProps.area < thresholdArea:
                continue

            pit3D = pit.worldGeometry
            centroid3D = sketch.sketchPoints.add(centroid).worldGeometry

            
            
            #x, y, z = getProfCenter(prof)
            dist = math.sqrt((centroid3D.x-pit3D.x)**2+(centroid3D.y-pit3D.y)**2+(centroid3D.z-pit3D.z)**2)
            #dist = math.sqrt((centroid.y-pit.geometry.z)**2+min((centroid.x-pit.geometry.x)**2, (centroid.x - pit.geometry.y)**2))
            #dist = centroid.y - pit.geometry.z

            #TODO COMPUTE DISTANCE BETTER
            #ui.messageBox("prof area: "+str(areaProps.area)+ " prof dist from pit: "+ str(dist)+ " cost: "+ str(dist+areaProps.area))
            if dist < 7 and (centroid3D.z > pit3D.z and not inTopMost(mainbody, rootComp, pit)):
                cost = areaProps.area
                if cost < minCost and areaProps.area > thresholdArea:
                    minCost=cost
                    bestProfOnPath = prof

            

            # ui.messageBox("distamce: "+str(dist))
            # if dist < 10:# and centroid.y > pit.geometry.z:
            #     ui.messageBox("found prof in range")
            #     profsInRange.add(prof)

    # ui.messageBox("sliceAlongPath profs in range found to be: "+str(len(profsInRange)))
            
    # minProf = None
    # minArea = sys.maxsize

    # for prof in profsInRange:
    #     areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
    #     area = areaProps.area
    #     if area < minArea:
    #         minProf = prof
    #         minArea = area
                
    # if minProf is None:
    #     ui.messageBox("minprof returned by slice along path is none")
    # return minProf
    if minCost > 80:
        return None

    return bestProfOnPath
    

def inTopMost(body: adsk.fusion.BRepBody, comp: adsk.fusion.Component, pit: adsk.fusion.SketchPoint):
    bodyHeight = body.boundingBox.maxPoint.z - body.boundingBox.minPoint.z
    upperThresh = body.boundingBox.maxPoint.z - 0.1 * bodyHeight

    if pit.worldGeometry.z > upperThresh:
        return True
    else:
        return False



def findArmpits(body:adsk.fusion.BRepBody, comp: adsk.fusion.Component, centroidSketch: adsk.fusion.Sketch):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    planes = comp.constructionPlanes
    sketches = rootComp.sketches

    centroids = centroidSketch.sketchPoints
    armpits = adsk.core.ObjectCollection.create()
    

    for point in centroids:
        #first point will be origin, skip this point
        if point.geometry.x == 0 and point.geometry.y == 0 and point.geometry.z == 0:
            continue
        
        #only start with points near the feet
        if point.geometry.z < (body.boundingBox.minPoint.z + 0.25*(body.boundingBox.maxPoint.z-body.boundingBox.minPoint.z)):
            curPoint = point
        else:
            continue

        for travPoint in centroids:
            curZ = curPoint.geometry.z


            if (travPoint.geometry.z == curZ + 1) and tightWithinRange(curPoint, travPoint):
                #ui.messageBox("point found, z: "+str(travPoint.geometry.z))
                #centroids.remove(curPoint)
                vector = adsk.core.Vector3D.create(0-curPoint.geometry.x, 0-curPoint.geometry.y, 0-curPoint.geometry.z)
                curPoint.move(vector)
                #curPoint.deleteMe()
                curPoint = travPoint
                continue 

            #if curPoint has not changed, it is an armpit
            # if travPoint.geometry.z - curZ > 8:
            #     armpits.add(curPoint)
            #     break
        if curPoint.geometry.z > (body.boundingBox.minPoint.z + 0.25*(body.boundingBox.maxPoint.z-body.boundingBox.minPoint.z)):
            armpits.add(curPoint)
            #ui.messageBox("added armpit, x: "+str(curPoint.geometry.x)+" y: "+str(curPoint.geometry.y)+" z: "+str(curPoint.geometry.z))


    return armpits


def tightWithinRange(point1: adsk.fusion.SketchPoint, point2: adsk.fusion.SketchPoint):
    factor = 4.5

    distance = math.sqrt((point2.geometry.x-point1.geometry.x)**2 + (point2.geometry.y-point1.geometry.y)**2)

    if distance < factor:
        return True
    else:
        return False


def traverseLeg(body:adsk.fusion.BRepBody, comp: adsk.fusion.Component, startPoint: adsk.core.Point3D):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    planes = comp.constructionPlanes
    sketches = rootComp.sketches

    #ui.messageBox("x: "+str(startPoint.x)+" y: "+str(startPoint.y)+" z: "+str(startPoint.z))

    sketch3D = sketches.add(rootComp.xYConstructionPlane)

    plane = getPlane(1 + startPoint.z, rootComp.xYConstructionPlane, rootComp)



def plotCentroids(body: adsk.fusion.BRepBody, comp: adsk.fusion.Component):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    planes = comp.constructionPlanes
    sketches = rootComp.sketches

    sketch3D = sketches.add(rootComp.xYConstructionPlane)

    ground = body.boundingBox.minPoint.z
    ceiling = body.boundingBox.maxPoint.z
    for i in range(math.ceil(ground), math.floor(ceiling)):
        #create plane
        plane = getPlane(i, comp.xYConstructionPlane, comp)
        sketch = getIntersectSketch(plane, body)
        #plot centroids
        for prof in sketch.profiles:
            areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
            if areaProps.area > thresholdArea:
                #plot centroid
                centroid = areaProps.centroid
                #newSketch = sketches.add(plane)
                point = adsk.core.Point3D.create(centroid.x, centroid.y, i)
                spoint = sketch3D.sketchPoints.add(point)

            

        sketch.deleteMe()
        plane.deleteMe()


    return sketch3D


def getPlane(offset, basePlane, comp):

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



def getIntersectSketch(plane, body, comp=adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    planes = rootComp.constructionPlanes
    sketches = comp.sketches

    sketch = sketches.add(plane)

    entities = []
    entities.append(body)


    sketch.intersectWithSketchPlane(entities)
    

    return sketch


def sliceByProf(mainbody: adsk.fusion.BRepBody, prof: adsk.fusion.Profile, pit: adsk.fusion.SketchPoint):
    
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    planes = rootComp.constructionPlanes
    sketches = rootComp.sketches

    # if prof is None:
    #     ui.messageBox("prof passed to slicebyProf is none")
    
    mainbody = findMainBody(rootComp)
    #create plane the profile lies on
    plane_input = planes.createInput()

    # Set the plane's orientation using the provided vector
    zeroValueInput = adsk.core.ValueInput.createByReal(0)
    plane_input.setByOffset(prof, zeroValueInput)

    # Create the plane
    plane = planes.add(plane_input)
    #plane = prof.plane
    

    areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
    centroid = areaProps.centroid

    preExistingBodies = adsk.core.ObjectCollection.create()
    for body in rootComp.bRepBodies:
        if body != mainbody:
            preExistingBodies.add(body)
    

    splitBodyFeats = rootComp.features.splitBodyFeatures
    splitBodyInput = splitBodyFeats.createInput(mainbody, plane, False)
    splitBodyFeats.add(splitBodyInput)

    mainbody = findMainBody(rootComp)

    # rejoin bodies that were split that are not in target profile

    validBodies = adsk.core.ObjectCollection.create()
    invalidBodies = adsk.core.ObjectCollection.create()

    #find face closest to pit
    minDist = sys.maxsize
    minDistBody = None

    for body in rootComp.bRepBodies:  #Change to split.bodies
        if body in preExistingBodies:
            continue

        if body == mainbody:
            # validBodies.add(body)
            # ui.messageBox("mainbody found")
            continue
        bounding_box = body.boundingBox
        
        
        faces = getFlatFaces(body)
        # ui.messageBox("facex: "+str(face.centroid.x)+" facey: "+str(face.centroid.y)+" facez: "+str(face.centroid.z))
        # ui.messageBox("centroidx: "+str(centroid.x)+" centroidy: "+str(centroid.y)+" centroidz: "+str(centroid.z))

        #ui.messageBox("distance: "+str(face.centroid.distanceTo(pit.geometry)))

        #encase in for face in faces
        for face in faces:

        #face = faces[0]
            pit3D = pit.worldGeometry
            #facePlane = getPlane(0, face, rootComp)
            sketch = sketches.add(rootComp.xYConstructionPlane)
            centroid3D = face.centroid
            #centroid3D = sketch.sketchPoints.add(centroid).worldGeometry

            dist = math.sqrt((centroid3D.x-pit3D.x)**2+(centroid3D.y-pit3D.y)**2+(centroid3D.z-pit3D.z)**2)
            #dist = centroid3D.distanceTo(pit)
            #ui.messageBox("dist from flat face to pit: "+str(dist))

            # if dist < 1:
            #     validBodies.add(body)
            #     minDist = dist
            #     continue

            if dist < minDist and (centroid3D.z >= pit3D.z-0.5 and not inTopMost(mainbody, rootComp, pit)):
                minDist = dist
                minDistBody = body

        # end of face in faces for loop

    # add valid and invalid bodies
                
    # validBodies.add(mainbody)
    # validBodies.add(minDistBody)
                

    for body in rootComp.bRepBodies:
        if body in preExistingBodies:
            continue

        if body == mainbody or body == minDistBody:
            
            validBodies.add(body)
            body.name = "valid"

        else:
            invalidBodies.add(body)
            #body.name = "invalid"

        

    #ui.messageBox("validBodies: "+str(len(validBodies))+ " invalidBodies: "+str(len(invalidBodies)))
    fuseInvalid(validBodies, invalidBodies)
    retryFuse(rootComp)


def retryFuse(rootComp: adsk.fusion.Component):
    largeBodies = adsk.core.ObjectCollection.create()
    tinyBodies = adsk.core.ObjectCollection.create()
    for body in rootComp.bRepBodies:
        if body.volume < thresholdVol:
            tinyBodies.add(body)
        else:
            largeBodies.add(body)

    if len(tinyBodies) >= 1:
        fuseInvalid(largeBodies, tinyBodies)


def fuseUnderBelly(rootComp: adsk.fusion.Component):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    features = rootComp.features

    mainbody = findMainBody(rootComp)
    #ui.messageBox("in fuse underbelly")

    for body in rootComp.bRepBodies:
        bbox = body.boundingBox

        xmax = bbox.maxPoint.x
        xmin = bbox.minPoint.x
        ymax = bbox.maxPoint.y
        ymin = bbox.minPoint.y

        if ((bbox.maxPoint.z-0.5 <= mainbody.boundingBox.minPoint.z and bbox.intersects(mainbody.boundingBox)) or
            (xmin < 0 < xmax and ymin < 0 < ymax and body != mainbody and bbox.intersects(mainbody.boundingBox))):
            #fuse body to mainbody
            targetBody = mainbody
            toolBodies = adsk.core.ObjectCollection.create()
            toolBodies.add(body)

            combineFeatures = features.combineFeatures
            combineInput = combineFeatures.createInput(targetBody, toolBodies)
            combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
            combineInput.isKeepToolBody = False

            combineFeature = combineFeatures.add(combineInput)

            #ui.messageBox("combine feature created")
        
    
def fuseSmallest(rootComp: adsk.fusion.Component):
    app = adsk.core.Application.get()
    ui  = app.userInterface
    features = rootComp.features

    minBody = None
    minVol = sys.maxsize

    #ui.messageBox("in fuseSmallest")

    for body in rootComp.bRepBodies:
        if body.volume < minVol:
            minBody = body
            minVol = body.volume

    toolBodies = adsk.core.ObjectCollection.create()
    toolBodies.add(minBody)
    targetBody = findMainBody(rootComp)

    combineFeatures = features.combineFeatures
    combineInput = combineFeatures.createInput(targetBody, toolBodies)
    combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    combineInput.isKeepToolBody = False

    combineFeature = combineFeatures.add(combineInput)

    # validBodies = adsk.core.ObjectCollection.create()
    # invalidBodies = adsk.core.ObjectCollection.create()

    # for body in rootComp.bRepBodies:
    #     if body == minBody:
    #         invalidBodies.add(body)
    #     else:
    #         validBodies.add(body)

    # ui.messageBox("validBodies: "+str(len(validBodies))+ " invalidbodies: "+str(len(invalidBodies)))

    # fuseInvalid(validBodies, invalidBodies)


def getFlatFaces(body: adsk.fusion.BRepBody):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    faces = adsk.core.ObjectCollection.create()

    for face in body.faces:
        if face.geometry.surfaceType == 0 and face.area > thresholdArea:
            faces.add(face)
    
    if len(faces) == 0:
        ui.messageBox("error: no planar face detected")
    
    return faces

def liesWithin(pit:adsk.fusion.SketchPoint, body: adsk.fusion.BRepBody):
    app = adsk.core.Application.get()
    ui  = app.userInterface

    bbox = body.boundingBox

    point = pit.geometry
    extent = 0.5

    # Define the corner points of the bounding box
    min_point = adsk.core.Point3D.create(point.x - extent, point.y - extent, point.z - extent)
    max_point = adsk.core.Point3D.create(point.x + extent, point.y + extent, point.z + extent)

    # Create the bounding box
    bounding_box = adsk.core.BoundingBox3D.create(min_point, max_point)

    if bbox.intersects(bounding_box):
        return True
    else:
        return False

    # ui.messageBox("pitx: "+str(pit.geometry.x)+" pity: "+str(pit.geometry.y)+" pitz: "+str(pit.geometry.z))
    # ui.messageBox("maxx: "+str(bbox.maxPoint.x)+" maxy: "+str(bbox.maxPoint.y)+" maxz: "+str(bbox.maxPoint.z))
    # ui.messageBox("minx: "+str(bbox.minPoint.x)+" miny: "+str(bbox.minPoint.y)+" minz: "+str(bbox.minPoint.z))

    # if (bbox.minPoint.x <= pit.geometry.x <= bbox.maxPoint.x and
    #     bbox.minPoint.y <= pit.geometry.y <= bbox.maxPoint.y and
    #     bbox.minPoint.z <= pit.geometry.z <= bbox.maxPoint.z):
    #     ui.messageBox("liesWithin returns true")
    #     return True
    # else:
    #     return False


def combineAllBodies(comp: adsk.fusion.Component):
    
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    planes = rootComp.constructionPlanes
    sketches = rootComp.sketches

    if len(comp.bRepBodies) == 1:
        return comp.bRepBodies[0]

    features = design.rootComponent.features

    combineFeatures = features.combineFeatures
    targetBody = comp.bRepBodies[0]
    toolBodies = adsk.core.ObjectCollection.create()

    for j in range(1, len(comp.bRepBodies)):
        toolBodies.add(comp.bRepBodies[j])
    
    combineInput = combineFeatures.createInput(targetBody, toolBodies)

    combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation

    combineInput.isKeepToolBody = False

    combineFeature = combineFeatures.add(combineInput)

    return comp.bRepBodies[0]
        


def fuseInvalid(validBodies, inValidBodies):

    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    planes = rootComp.constructionPlanes

    features = design.rootComponent.features
    targetBody = None

    for invalidBody in inValidBodies:
        toolBodies = adsk.core.ObjectCollection.create()
        toolBodies.add(invalidBody)
        invalidBBox = invalidBody.boundingBox

        for validBody in validBodies:
            validBBox = validBody.boundingBox
            if invalidBBox.intersects(validBBox):
                #ui.messageBox("invalid intersects valid")
                targetBody = validBody


        if targetBody == None:
            ui.messageBox("ERROR: no targetBody found")
        else:
            combineFeatures = features.combineFeatures
            combineInput = combineFeatures.createInput(targetBody, toolBodies)
            combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
            combineInput.isKeepToolBody = False

            combineFeature = combineFeatures.add(combineInput)

            
    
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
    
            
def getProfCenter(prof: adsk.fusion.Profile, comp: adsk.fusion.Component):
    
    app = adsk.core.Application.get()
    ui  = app.userInterface

    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent
    planes = rootComp.constructionPlanes

    sketch = prof.parentSketch
    
    # Get the sketch plane
    sketch_plane = prof.plane

    planes = comp.constructionPlanes

    planeInput = planes.createInput()

    planeInput.setByPlane(prof.plane)
    planeOne = planes.add(planeInput)

    # Get the bounding box of the profile
    bbox = prof.boundingBox

    # Compute the center of the bounding box
    center = adsk.core.Point3D.create(
        (bbox.minPoint.x + bbox.maxPoint.x) / 2,
        (bbox.minPoint.y + bbox.maxPoint.y) / 2,
        (bbox.minPoint.z + bbox.maxPoint.z) / 2
    )

    # Convert the center to world coordinates
    #center.transformBy(planeOne.geometry.)

    return center.x, center.y, center.z
