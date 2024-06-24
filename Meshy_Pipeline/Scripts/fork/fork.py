#Author-
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        ui.messageBox('Hello script')

        parentDesign = app.activeProduct
        parentRootComp = parentDesign.rootComponent

        #save_and_open_document()

        docParent = app.activeDocument

        docChild = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)

        #docParent.copyTo(docChild)

        childDesign = app.activeProduct
        #childDesign.rootComponent = parentRootComp
        childRootComp = childDesign.rootComponent

        childRootComp.occurrences.addByInsert(docParent.dataFile, adsk.core.Matrix3D.create(), True)


        ui.messageBox("Call ELM, Export URDF, then close")

        #docChild.close(False)

        # i=0

        # for occurrence in parentRootComp.allOccurrences:
        #     childRootComp.occurrences.addExistingComponent(occurrence.component, adsk.core.Matrix3D.create())
        #     i=i+1

        # for source_comp in parentDesign.allComponents:
        #     # Create a new component in the new design.
        #     if source_comp == parentRootComp:
        #         continue

            
        #     new_comp = childRootComp.occurrences.addNewComponentCopy(source_comp, adsk.core.Matrix3D.create())
        #     #new_comp.component = source_comp
            

        

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))



def save_and_open_document():
    try:
        # Get the active Fusion 360 application and document.
        app = adsk.core.Application.get()
        design = app.activeDocument

        # Save the current document to a temporary location.
        temp_file_path = adsk.fusion.SystemUtil.tempFolder + design.name
        design.saveAs(temp_file_path)

        # Create a new document.
        new_design = app.documents.add(adsk.fusion.DesignDocumentTypes.DesignDocumentType)

        # Import the saved document into the new document.
        new_design.importToDesign(temp_file_path)

        # Optionally, you can delete the temporary file after importing.
        # adsk.fusion.SystemUtil.deleteFile(temp_file_path)

    except Exception as ex:
        # Handle any exceptions that occur during the process.
        adsk.core.Application.get().userInterface.messageBox('Failed to save and open document: {}'.format(str(ex)))
