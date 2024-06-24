# Text2Robot

## Overview
Text2Robot automatically converts a text prompt to a quadrupedal robot. We utilize a state of the art text to mesh generative model as initialization for our pipeline, and convert the static mesh to a kinetic robot model. We evolve the robots control and morphology simultaneously using our evolutionary algorithm.

## Text2Mesh
We use the Meshy website https://www.meshy.ai/ to generate STL meshes from text prompts. STL's used in our experiments are provided in RobotMakingRobots/MeshyPipeline/Meshystl's. 

![MeshyHomeScreen](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/0fa40918-f86d-4850-900c-a0971a081f2c)



Additional meshes can be generated using Meshy, although they are not guaranteed to work with the provided slicing script. Include the keywords "quadrupedal walking robot" in the text prompt for best results.

![MeshyText](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/72d1d86f-b40e-47d9-823a-877393302f4b)


## Mesh2CAD

STL meshes can be converted to a fusion360 assembly using the provided sliceBody script. First open Fusion360, and preprocess the generated or downloaded mesh desired. Installation instructions can be found at https://www.autodesk.com/campaigns/education/fusion-360. Insert the mesh into a fusion360 document, and use the convert mesh operation to convert the mesh to a brepboy. Organic mesh conversion is enabled with the fusion design extension.

![InsertMesh](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/6260c319-2e28-4328-9282-bf41ddf6e99f)
![ConvertMesh](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/752ee300-753c-41fc-bc06-1f0e3ca92f7b)

To slice the brepbody, add the wrapper script and installNumpy script to fusion360 scripts and add ins. Click on the green plus, and navigate to RobotsMakingRobots/MeshyPipeline/Scripts/ and click the wrapper or installNumpy folder.

![AddScripts](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/761fe4d8-8e56-4d94-aae2-a468c2e5c465)


Run installNumpy to install necessary python libraries to the same file path as the wrapper script. Running the wrapper script will convert the preprocessed brepbody to a robot assembly. If the mesh does not properly slice, adjusting the steps in the slicebyDX function of slicebody can adjust the location of shoulder slices.

![AdjustSteps](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/22e2407d-51d0-4c4e-9bb2-3220d6cb2993)


## CAD2URDF

Uncommenting the URDF exporter function in wrapper will export the generated robot to a URDF. Uncommenting the loop will create 30 variations of the generated robot, and export all as URDF's. This may take up to 15 minutes.

![URDFExporter](https://github.com/generalroboticslab/RobotsMakingRobots/assets/46581478/2199d1c2-33e8-4c08-9eb5-a24ddb90e0e1)



## Evolutionary loop
