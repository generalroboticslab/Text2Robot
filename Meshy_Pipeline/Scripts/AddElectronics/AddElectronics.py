#Author-
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback


import os

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        ui.messageBox('running')

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        rootComp = design.rootComponent
        sketches = rootComp.sketches
        planes = rootComp.constructionPlanes
        axes = rootComp.constructionAxes



        meshdir = "C:\Users\ringe\Spring2024Research\URDF Tests\RobotDog\meshes"
        
        for filename in os.listdir(meshdir):

        #for file in specified folder:

            #open stl, convert to organic brep
            import_mesh(rootComp, filename)
            convert_to_brep(app, ui)

            break


            #find flat faces, add motor box

            #if baselink, also make electronics box

            #make finalmeshes dir, export final stl into folder to be sliced
        

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def import_mesh(root, filename):
    """Import mesh from file, add to design, and apply modifications."""
    baseFeatures = root.features.baseFeatures
    baseFeature = baseFeatures.add()
    baseFeature.startEdit()
    importMeshTechnical(filename, root.meshBodies, baseFeature)
    baseFeature.finishEdit()

def importMeshTechnical(path, meshBodies, baseFeature):
    """Import mesh from a specified path and add it to the design."""
    unitCm = adsk.fusion.MeshUnits.CentimeterMeshUnit
    meshList = meshBodies.add(path, unitCm, baseFeature)
    return [mesh for mesh in meshList]

def convert_to_brep(app, ui):
    des  :adsk.fusion.Design = app.activeProduct
    root :adsk.fusion.Component = des.rootComponent
    # select meshBodies
    sels :adsk.core.Selections = ui.activeSelections
    sels.clear()
    for mesh in root.meshBodies:
        sels.add(mesh)

    # Executing a command
    # It seems to run regardless of Direct / Parametric.
    app.executeTextCommand(u'Commands.Start ParaMeshRepairCommand')
    app.executeTextCommand(u'NuCommands.CommitCmd')

    for mesh in root.meshBodies:
        sels.add(mesh)

    app.executeTextCommand(u'Commands.Start ParaMeshConvertCommand')
    app.executeTextCommand(u'NuCommands.CommitCmd')


def getPlane(offset, basePlane):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        rootComp = design.rootComponent
        planes = rootComp.constructionPlanes

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


