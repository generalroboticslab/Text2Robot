# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
import os.path, re
from xml.etree import ElementTree
from xml.dom import minidom

import os
import re
import glob
# pip install pycollada
# pip instal open3d
import open3d as o3d
import trimesh
import numpy as np


def copy_occs(root):    
    """    
    duplicate all the components
    """    
    def copy_body(allOccs, occs):
        """    
        copy the old occs to new component
        """
        
        bodies = occs.bRepBodies
        transform = adsk.core.Matrix3D.create()
        
        # Create new components from occs
        # This support even when a component has some occses. 

        new_occs = allOccs.addNewComponent(transform)  # this create new occs
        if occs.component.name == 'base_link':
            occs.component.name = 'old_component'
            new_occs.component.name = 'base_link'
        else:
            new_occs.component.name = re.sub('[ :()]', '_', occs.name)
        new_occs = allOccs[-1]
        for i in range(bodies.count):
            body = bodies.item(i)
            body.copyToComponent(new_occs)
    
    allOccs = root.occurrences
    oldOccs = []
    coppy_list = [occs for occs in allOccs]
    for occs in coppy_list:
        if occs.bRepBodies.count > 0:
            copy_body(allOccs, occs)
            oldOccs.append(occs)

    for occs in oldOccs:
        occs.component.name = 'old_component'


def export_stl(design, save_dir, components):  
    """
    export stl files into "sace_dir/"
    
    
    Parameters
    ----------
    design: adsk.fusion.Design.cast(product)
    save_dir: str
        directory path to save
    components: design.allComponents
    """
          
    # create a single exportManager instance
    exportMgr = design.exportManager
    # get the script location
    try: os.mkdir(save_dir + '/meshes')
    except: pass
    scriptDir = save_dir + '/meshes'  
    # export the occurrence one by one in the component to a specified file
    for component in components:
        allOccus = component.allOccurrences
        for occ in allOccus:
            if 'old_component' not in occ.component.name:
                try:
                    print(occ.component.name)
                    fileName = scriptDir + "/" + occ.component.name              
                    # create stl exportOptions
                    stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
                    stlExportOptions.sendToPrintUtility = False
                    stlExportOptions.isBinaryFormat = True
                    # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
                    stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                    exportMgr.execute(stlExportOptions)

                    #Unsimplified mesh stored in scriptDir with filename
                    #create simplified mesh folder

                    



                except:
                    print('Component ' + occ.component.name + 'has something wrong.')


def simplify_meshes(save_dir):
    src_dir = save_dir + '/meshes'
    dest_dir = save_dir + '/simple_meshes'

    decimation = 100 # reduce triangles by this factor

    src_ext = "stl"
    # dest_ext = "stl"

    # Compile a case-insensitive regular expression pattern for the extension
    pattern = re.compile(re.escape(src_ext), re.IGNORECASE)

    # Use glob to recursively find all files in the folder and its subfolders
    paths = glob.glob(os.path.join(src_dir, '**'), recursive=True)

    # Filter the files based on the case-insensitive extension pattern
    paths = [file for file in paths if pattern.search(file)]

    # Calculate the relative path between the original path and the matching path
    relative_paths = [os.path.relpath(path, src_dir) for path in paths]

    # # Print the list of files
    # for path in relative_paths:
    #     print(path)
        

    for path, rel_path in zip(paths,relative_paths):
        
        print(f"loading {rel_path}")
        o3d_mesh = load_to_o3d_mesh(path)
        print(o3d_mesh)
        # print(f"watertight={o3d_mesh.is_watertight()}")
        # o3d.visualization.draw_geometries([o3d_mesh])
        mesh_export = o3d_mesh
        
        mesh_export = simplify_o3d_mesh(o3d_mesh,decimation)
        # export
        # export_path = f"{dest_dir}/{rel_path}.{dest_ext}"
        export_path = f"{dest_dir}/{rel_path}"
        
        
        # Get the folder path of the file path
        folder_path = os.path.dirname(export_path)
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        o3d.io.write_triangle_mesh(export_path,mesh_export,write_vertex_normals=True)
        # o3d.visualization.draw_geometries([mesh_export])
        print(f"saved to \"{export_path}\"")
        file_size_bytes_original = os.path.getsize(path)
        file_size_bytes_export = os.path.getsize(export_path)
        print(f"reduction ratio={(1-file_size_bytes_export/file_size_bytes_original)*100:.1f}%")
        print("-"*10)

def fix_mesh(mesh_file):
    # Load your Trimesh mesh
    trimesh_mesh = trimesh.load(mesh_file)  

    # Repair the mesh
    trimesh.repair.fill_holes(trimesh_mesh)

    # Extract vertices and triangles
    vertices = trimesh_mesh.vertices.astype(np.float64)
    triangles = trimesh_mesh.faces

    # Create Open3D TriangleMesh
    open3d_mesh = o3d.geometry.TriangleMesh()
    open3d_mesh.vertices = o3d.utility.Vector3dVector(vertices)
    open3d_mesh.triangles = o3d.utility.Vector3iVector(triangles)

    # Compute vertex normals (if needed)
    open3d_mesh.compute_vertex_normals()
    open3d_mesh.compute_triangle_normals()
    
    return open3d_mesh


def load_to_o3d_mesh(path:str):
    if path.endswith('.dae'): # dae mesh
        import collada
        c_mesh = collada.Collada(path)
        o3d_mesh = o3d.geometry.TriangleMesh()
        for geom in c_mesh.geometries:
            print(geom)
            for triset in geom.primitives:
                if not type(triset) is collada.triangleset.TriangleSet:
                    continue
                print(geom,triset)
            
                o3d_mesh += o3d.geometry.TriangleMesh(
                    o3d.utility.Vector3dVector(triset.vertex), # vertices
                    o3d.utility.Vector3iVector(triset.vertex_index) # triangles
                    )
    else: # stl/obj
        o3d_mesh = o3d.io.read_triangle_mesh(path)
        
    if o3d_mesh.is_watertight()==False: # fix water tightness
        trimesh_mesh = trimesh.load(path) # Load your Trimesh mesh
        # Repair the mesh
        print("mesh is not water tight, tyring to rill holes")
        if trimesh.repair.fill_holes(trimesh_mesh) is False:
            raise(NotImplementedError("cannot fix broken mesh"))
        # Create Open3D TriangleMesh
        o3d_mesh = o3d.geometry.TriangleMesh()
        o3d_mesh.vertices = o3d.utility.Vector3dVector(trimesh_mesh.vertices.astype(np.float64))
        o3d_mesh.triangles = o3d.utility.Vector3iVector(trimesh_mesh.faces)
        
    o3d_mesh.compute_triangle_normals()
    o3d_mesh.compute_vertex_normals()
    return o3d_mesh

def simplify_o3d_mesh(o3d_mesh:o3d.geometry.TriangleMesh, decimation:int,maximum_error=0.0005):
    if decimation<=1:
        return o3d_mesh
    mesh_smp = o3d_mesh\
        .remove_unreferenced_vertices()\
        .remove_duplicated_vertices()\
        .remove_duplicated_triangles()\
        .remove_degenerate_triangles()\
        .remove_non_manifold_edges()\
        .merge_close_vertices(maximum_error)\
        .simplify_vertex_clustering(maximum_error)\
        .simplify_quadric_decimation(
            target_number_of_triangles=int(len(o3d_mesh.triangles)/decimation),
            maximum_error = maximum_error
            )
    mesh_smp.compute_vertex_normals()
    mesh_smp.compute_triangle_normals()
    print(mesh_smp)
    return mesh_smp
                

def file_dialog(ui):     
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog' 
    
    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into 
    that about center of mass coordinate


    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]
    
    
    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                         -x*y, -y*z, -x*z]
    return [ i - mass*t for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element
    
    
    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

