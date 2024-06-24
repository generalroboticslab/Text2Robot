import adsk.core, adsk.fusion, adsk.cam, traceback
import os, requests, random, time, math
# from .utils import SliceBody3

# Global variables for application and user interface
app = adsk.core.Application.cast(None)
ui = adsk.core.UserInterface.cast(None)

def load_secret(filename):
    """Load and return the secret string from a file."""
    with open(filename, 'r') as file:
        secret = file.read().strip()  # Removes any leading/trailing whitespace
    return secret

def run(context):
    global app, ui
    try:
        # Initialize Fusion 360 application and user interface
        app = adsk.core.Application.get()
        ui = app.userInterface

        product = app.activeProduct

        design = adsk.fusion.Design.cast(product)

        ui.messageBox("start run")

        ui.inputBox("Enter a prompt for the Generative AI:")

        # # TODO: Update directory_path to where your specific GenerateRobot folder is stored
        # directory_path = 'C:/Users/ringe/Spring2024Research/RobotsMakingRobots/Meshy Pipeline/Scripts/GenerateRobot'
        # os.chdir(directory_path)

        # # Run the test print script
        # # testPrint.run(context)

        # # Get user input for the AI generative prompt
        # prompt, cancelled = ui.inputBox("Enter a prompt for the Generative AI:")
        # if cancelled:
        #     return

        # # Load the API key from a file
        # secret_file = 'api_key.txt'
        # API_KEY = load_secret(secret_file)

        # # Setup payload for the API request
        # negative_prompt = "low quality, low resolution, low poly, ugly, thin, brittle"
        # random_seed = random.randint(1, 2**30)
        # payload = {
        #     "mode": "preview",
        #     "prompt": prompt,
        #     "art_style": "realistic",
        #     "negative_prompt": negative_prompt,
        #     "seed": random_seed
        # }

        # # Make the initial API call to generate the model
        # generation_response = make_api_call("https://api.meshy.ai/v2/text-to-3d", API_KEY, payload)
        # task_id = str(generation_response.json()['result'])

        # # Poll for the status of the model generation
        # status, obj_url = poll_model_status(task_id, API_KEY)

        # # Download the generated model
        # download_model(obj_url, prompt)

        # # Add the model to a Fusion 360 document
        # add_model_to_document(prompt)

        # scale_mesh(app, 10) # Scale mesh with ratio of 8

        # convert_to_brep(app, ui) # Convert to brep (for organic must pre-select)

        # rotate_x_axis_90_degrees(app) # Rotates body

        #SliceBody3.run(context)

    except Exception as e:
        if ui:
            ui.messageBox(f'Failed:\n{traceback.format_exc()}')

def scale_mesh(app, scale_ratio):
    des  :adsk.fusion.Design = app.activeProduct
    root :adsk.fusion.Component = des.rootComponent
    # check meshBodies count
    if root.meshBodies.count < 1:
        ui.messageBox('The RootComponent does not have a MeshBody.')
        return

    scales = root.features.scaleFeatures
    scaleFactor = adsk.core.ValueInput.createByReal(scale_ratio)
    inputColl = adsk.core.ObjectCollection.create()
    #add body
    inputColl.add(root.meshBodies.item(0))
    scaleInput = scales.createInput(inputColl, root.originConstructionPoint, scaleFactor)
    #Scale body to targetVolume
    scale = scales.add(scaleInput)

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

def make_api_call(url, api_key, payload=None):
    """Send a POST request to the specified URL with the given payload and headers."""
    headers = {"Authorization": f"Bearer {api_key}"}
    response = requests.post(url, headers=headers, json=payload)
    response.raise_for_status()
    return response

def poll_model_status(task_id, api_key):
    """Poll the API for the status of the model generation task."""
    headers = {"Authorization": f"Bearer {api_key}"}
    status = 'PENDING'
    ui.messageBox("Starting Meshy Job")
    obj_url = None  # Initialize to None, will update upon completion

    while status in ['PENDING', 'IN_PROGRESS']:
        time.sleep(1)
        object_response = requests.get(f"https://api.meshy.ai/v2/text-to-3d/{task_id}", headers=headers)
        object_response.raise_for_status()
        status = object_response.json()['status']
        if status not in ['PENDING', 'IN_PROGRESS']:
            obj_url = object_response.json()['model_urls']['obj']
            break  # Exit loop if status is COMPLETED

    # Ensure we return a tuple even if status isn't COMPLETED (e.g., failed or error states)
    return status, obj_url

def download_model(obj_url, prompt):
    """Download the model from the provided URL and save it locally."""
    name = prompt.replace(" ", "_")
    filename = f"models/{name}.obj"
    response = requests.get(obj_url)

    if response.status_code == 200:
        with open(filename, 'wb') as file:
            file.write(response.content)
        ui.messageBox(f"Model '{filename}' downloaded successfully.")
    else:
        ui.messageBox(f"Failed to download the model. Status code: {response.status_code}")

def add_model_to_document(prompt):
    """Add the downloaded model to a new Fusion 360 document."""
    filename = f"models/{prompt.replace(' ', '_')}.obj"
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    # Import the model and perform operations
    import_mesh(root, filename)

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

def rotate_x_axis_90_degrees(app):
        des  :adsk.fusion.Design = app.activeProduct
        root :adsk.fusion.Component = des.rootComponent

        bodies = root.bRepBodies
        if bodies.count == 0:
            ui.messageBox('No BRep body found in the root component.')
            return
        body = bodies.item(0)

        # Create an ObjectCollection for the body to move
        bodiesToMove = adsk.core.ObjectCollection.create()
        bodiesToMove.add(body)

        # Create a transform to rotate 90 degrees about the X-axis
        # Fusion 360 API uses radians, so convert degrees to radians
        degrees = 90
        radians = degrees * (math.pi / 180)

        # Create the rotation matrix
        axis = adsk.core.Vector3D.create(1.0, 0.0, 0.0) # Rotation around X-axis
        point = adsk.core.Point3D.create(0, 0, 0) # Rotation around origin
        matrix = adsk.core.Matrix3D.create()
        matrix.setToRotation(radians, axis, point)

        # Create the move feature
        moveFeatures = root.features.moveFeatures
        moveFeatureInput = moveFeatures.createInput(bodiesToMove, matrix)
        moveFeature = moveFeatures.add(moveFeatureInput)