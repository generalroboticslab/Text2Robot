#Author-
#Description-

from PIL import Image
import adsk.core, adsk.fusion, adsk.cam, traceback
import os, sys
import numpy as np
from scipy.signal import argrelextrema


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        #ui.messageBox(os.path.abspath(os.getcwd()) + "/csvs/cross_" + str(app.activeDocument.dataFile.name) + ".csv")
        ui.messageBox("running")
        
        number_of_legs = 4

        #doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        # Get the root component of the active design
        rootComp = design.rootComponent

        # Create sketch
        sketches = rootComp.sketches

        #________________________________________________________________________

        #prof = sketch.profiles.item(0)
        # Get construction planes
        planes = rootComp.constructionPlanes

        boundingBox = rootComp.boundingBox
        maxPoint = boundingBox.maxPoint
        minPoint = boundingBox.minPoint

        planeOff = round(max(maxPoint.x, maxPoint.y, maxPoint.z))

        #create array for yz cross sections
        yzarr = np.zeros((2*planeOff*10 + 1, 4))

        #pathname = "C:Users/ringe/Spring2024Research/RobotsMakingRobots/Meshy Pipeline/scripts/csvs/"

        #np.savetxt(pathname+"cross_"+str(app.activeDocument.dataFile.name)+".csv", yzarr, delimiter=",")

        yzstep = (maxPoint.x - minPoint.x) / 31
        #for loop across YZ plane
        for i in range(30):
            planeInput = planes.createInput()
            offsetValue = adsk.core.ValueInput.createByReal(minPoint.x + (1+i)*yzstep)
            planeInput.setByOffset(rootComp.yZConstructionPlane, offsetValue)
            planeOne = planes.add(planeInput)

        
        
            # Get the health state of the plane
            health = planeOne.healthState
            if health == adsk.fusion.FeatureHealthStates.ErrorFeatureHealthState or health == adsk.fusion.FeatureHealthStates.WarningFeatureHealthState:
                message = planeOne.errorOrWarningMessage

            #create sketch on plane one
            sketch = sketches.add(planeOne)

            entities = []
            entities.append(design.rootComponent.bRepBodies[0])

            sketch.intersectWithSketchPlane(entities)

            #select slice

            
            #measure
            crossArea = 0

            for prof in sketch.profiles:

                areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        
                # Get area
                area = areaProps.area

                crossArea = crossArea + area

            #yzarr[i][0] = crossArea

            sketch.deleteMe()
            planeOne.deleteMe()

        xzstep = (maxPoint.y - minPoint.y) / 31

        xzarr = np.zeros(30)
        #XZ PLane
        for i in range(30):
            planeInput = planes.createInput()
            offsetValue = adsk.core.ValueInput.createByReal(minPoint.y + (1+i)*xzstep)
            planeInput.setByOffset(rootComp.xZConstructionPlane, offsetValue)
            planeOne = planes.add(planeInput)

        
        
            # Get the health state of the plane
            health = planeOne.healthState
            if health == adsk.fusion.FeatureHealthStates.ErrorFeatureHealthState or health == adsk.fusion.FeatureHealthStates.WarningFeatureHealthState:
                message = planeOne.errorOrWarningMessage

            #create sketch on plane one
            sketch = sketches.add(planeOne)

            entities = []
            entities.append(design.rootComponent.bRepBodies[0])

            sketch.intersectWithSketchPlane(entities)

            #select slice

            
            #measure
            crossArea = 0

            for prof in sketch.profiles:

                areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        
                # Get area
                area = areaProps.area

                crossArea = crossArea + area

            #yzarr[i][1] = crossArea

            xzarr[i] = crossArea

            sketch.deleteMe()
            planeOne.deleteMe()

        #XY PLane
        xystep = (maxPoint.z - minPoint.z) / 31
        xyarr = np.zeros(30)
        for i in range(30):
            planeInput = planes.createInput()
            offsetValue = adsk.core.ValueInput.createByReal(minPoint.z + (1+i)*xystep)
            planeInput.setByOffset(rootComp.xYConstructionPlane, offsetValue)
            planeOne = planes.add(planeInput)

        
        
            # Get the health state of the plane
            health = planeOne.healthState
            if health == adsk.fusion.FeatureHealthStates.ErrorFeatureHealthState or health == adsk.fusion.FeatureHealthStates.WarningFeatureHealthState:
                message = planeOne.errorOrWarningMessage

            #create sketch on plane one
            sketch = sketches.add(planeOne)

            entities = []
            entities.append(design.rootComponent.bRepBodies[0])

            sketch.intersectWithSketchPlane(entities)

            #select slice

            
            #measure
            crossArea = 0

            for prof in sketch.profiles:

                areaProps = prof.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        
                # Get area
                area = areaProps.area

                crossArea = crossArea + area

            #yzarr[i][2] = crossArea
            xyarr[i] = crossArea

            sketch.deleteMe()
            planeOne.deleteMe()


        #create 4th col for csv
        val = -planeOff
        for i in range(2*planeOff*10 + 1):
            #yzarr[i][3] = val
            val = val + 0.1

        
        xzfilename = "C:/Users/ringe/Spring2024Research/xzgradient.PNG"
        xyfilename = "C:/Users/ringe/Spring2024Research/xygradient.PNG"

        xzimg = np.tile(xzarr, 200)
        xyimg = np.tile(xyarr, 200)

        normalized_array = 255 * (xzarr - np.min(xyarr)) / (np.max(xyarr) - np.min(xyarr))
        normalized_array = normalized_array.astype(np.uint8)
        
        # Convert the array to a PIL image
        image = Image.fromarray(normalized_array)
        
        # Save the image
        image.save(xyfilename)

        ui.messageBox("image saved")

        #export yzarr to csv file somewhere
        #np.savetxt("C:/Users/ringe/Spring2024Research/cross_" + str(app.activeDocument.dataFile.name) + ".csv", yzarr, delimiter=",")

        #____________________________________________________________________________________-


        #from array, compute joint locations:

        #assume forward +x direction
        #slice mins/important section of XZ plane
        #count bodies


        #Find symmetry in XZ plane
        #find minima of XZ; yzarr[1]
        #x = argrelextrema(yzarr[:, 1], np.less)
        #ui.messageBox(str(x))




    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))