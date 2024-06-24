import adsk.core, adsk.fusion, adsk.cam, traceback
import os, sys

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # install_numpy = sys.path[0] +'\Python\python.exe -m pip install numpy'
        # install_scipy = sys.path[0] +'\Python\python.exe -m pip install scipy'
        # install_requests = sys.path[0] +'\Python\python.exe -m pip install requests'
        # install_yaml = sys.path[0] +'\Python\python.exe -m pip install pyyaml'
        # install_open3d = sys.path[0] +'\Python\python.exe -m pip install open3d'
        # install_trimesh = sys.path[0] +'\Python\python.exe -m pip install trimesh'
        # install_pycollada = sys.path[0] +'\Python\python.exe -m pip install pycollada'

        # os.system('cmd /c "' + install_numpy + '"')
        # os.system('cmd /c "' + install_scipy + '"')
        # os.system('cmd /c "' + install_open3d + '"')
        # os.system('cmd /c "' + install_trimesh + '"')
        # os.system('cmd /c "' + install_pycollada + '"')
        # os.system('cmd /c "' + install_requests + '"')
        # os.system('cmd /c "' + install_yaml + '"')
        
        try:
            import trimesh
            # import requests
            ui.messageBox("trimesh Success!")
        except:
            ui.messageBox("trimesh Failed")
        
        try:
            import collada
            # import requests
            ui.messageBox("Collada Success!")
        except:
            ui.messageBox("Collada Failed")
        
        try:
            import open3d as o3d
            # import requests
            ui.messageBox("Open3D Success!")
        except:
            ui.messageBox("Open3D Failed")     

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))