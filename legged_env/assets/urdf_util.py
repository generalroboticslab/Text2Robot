import trimesh
import open3d as o3d
# import urdfpy
import yourdfpy
import numpy as np
import os
import shutil
from functools import partial
from itertools import chain
import copy

def get_urdf_bounding_box(urdf:yourdfpy.URDF):
    """return urdf bounding box
    Example:
        urdf_path=....
        urdf = yourdfpy.URDF.load(urdf_path)
        bounding_box = get_urdf_bounding_box(urdf)
    """
    scene = urdf.scene
    bounding_box = scene.bounding_box
    return bounding_box

def vf_to_mesh(vertices, faces, return_type="o3d"):
    """Converts vertex and face arrays into a mesh object.

    Args:
        vertices: NumPy array (n, 3) of vertex coordinates.
        faces: NumPy array (m, 3) of vertex indices forming triangles.
        return_type: The type of mesh object to return ("o3d" for Open3D, "trimesh" for Trimesh). Defaults to "o3d".

    Returns:
        The mesh object of the specified type:
            - open3d.geometry.TriangleMesh if `return_type` is "o3d".
            - trimesh.Trimesh if `return_type` is "trimesh".

    Raises:
        ValueError: If `return_type` is not "o3d" or "trimesh".
    """
    if return_type == "o3d":
        return o3d.geometry.TriangleMesh(
            vertices=o3d.utility.Vector3dVector(vertices),
            triangles=o3d.utility.Vector3iVector(faces),
        )
    elif return_type == "trimesh":
        return trimesh.Trimesh(vertices=vertices, faces=faces)
    else:
        raise ValueError("Invalid return_type. Choose 'o3d' or 'trimesh'.")


def triangles_to_vf(triangles):
    """Extracts unique vertices and faces from a NumPy array of triangles.

    Args:
        triangles: NumPy array (n, 3, 3) of triangles, where each triangle is 
                   defined by 3 vertices with (x, y, z) coordinates.

    Returns:
        Tuple (vertices, faces):
            - vertices: NumPy array (num_unique_vertices, 3) of unique vertex coordinates.
            - faces: NumPy array (n, 3) of vertex indices corresponding to the unique vertices.

    Example:
        ```python
        import yourdfpy

        urdf = yourdfpy.URDF.load(...)  
        scene = get_urdf_scene(urdf) 
        vertices, faces = triangles_to_vf(scene.triangles)  
        mesh = vf_to_mesh(vertices, faces, return_type="trimesh")
        mesh.show()
        ```
    """

    # Flatten the triangles array and find unique vertices
    vertices, inverse_indices = np.unique(
        triangles.reshape(-1, 3), axis=0, return_inverse=True
    )

    # Reshape the inverse indices to get the face indices for the unique vertices
    faces = inverse_indices.reshape(-1, 3)

    return vertices, faces


def scene_to_vf(scene: trimesh.Scene):
    """Extracts and transforms vertices and faces from a trimesh Scene.

    Args:
        scene: The trimesh.Scene object to process.

    Returns:
        Tuple (vertices, faces):
            - vertices: NumPy array (num_vertices, 3) of transformed vertex coordinates.
            - faces: NumPy array (num_faces, 3) of vertex indices.

    Example:
        ```python
        import yourdfpy

        urdf = yourdfpy.URDF.load(...)  
        scene = get_urdf_scene(urdf) 
        vertices, faces = scene_to_vf(scene)  
        mesh = vf_to_mesh(vertices, faces, return_type="o3d")
        o3d.visualization.draw_geometries([mesh])  # Visualize using Open3D
        ```
    """
    vertices_list = []
    faces_list = []
    offset = 0

    for node_name in scene.graph.nodes_geometry:
        # Get transform and geometry for the current node
        transform, geometry_name = scene.graph[node_name]
        geometry = scene.geometry[geometry_name]

        # Skip if geometry doesn't have triangles
        if not hasattr(geometry, "triangles"):
            continue

        # Apply transform to vertices and append to list
        transformed_vertices = trimesh.transformations.transform_points(
            geometry.vertices.copy(), matrix=transform
        )
        vertices_list.append(transformed_vertices)

        # Append face indices with offset and update offset
        faces_list.append(geometry.faces + offset)
        offset += geometry.vertices.shape[0]

    # Stack vertices and faces into single arrays
    vertices = np.vstack(vertices_list)
    faces = np.vstack(faces_list)
    return vertices, faces


def write_urdf(urdf, new_path, old_path, copy_mesh=False):
    """Writes a URDF file, optionally copying mesh files and adjusting paths.
    
    Args:
        urdf: the URDF object to write.
        new_path: The path to write the new URDF file to.
        old_path: The path of the original URDF file.
        copy_mesh: If True, mesh files are copied and paths adjusted in the new URDF.
    """
    # --- Path Handling ---
    new_dir, new_urdf_name = os.path.split(os.path.abspath(new_path))
    old_dir, _ = os.path.split(os.path.abspath(old_path))  # Old URDF name not used here
    rel_dir = os.path.relpath(old_dir, new_dir)

    os.makedirs(new_dir, exist_ok=True)  # Create directory if it doesn't exist

    # --- Mesh Copying (if enabled) ---
    if copy_mesh:
        new_urdf = copy.deepcopy(urdf)
        new_mesh_dir_rel = "meshes"
        new_mesh_dir = os.path.join(new_dir, new_mesh_dir_rel)
        os.makedirs(new_mesh_dir, exist_ok=True)

        mesh_set = set() 
        for link in new_urdf.link_map.values():
            for v in chain(link.collisions, link.visuals):
                if v.geometry and v.geometry.mesh.filename not in mesh_set:
                    # Copy the mesh file and update the path in the URDF
                    old_mesh_path = os.path.join(old_dir, v.geometry.mesh.filename)
                    new_mesh_path = os.path.join(new_mesh_dir, os.path.basename(v.geometry.mesh.filename))
                    shutil.copy2(old_mesh_path, new_mesh_path)
                    v.geometry.mesh.filename = os.path.join(new_mesh_dir_rel, os.path.basename(v.geometry.mesh.filename))
                    mesh_set.add(v.geometry.mesh.filename) 

        # Set filename handler to null to avoid errors
        prev_filename_handler = new_urdf._filename_handler
        new_urdf._filename_handler = yourdfpy.filename_handler_null 

    # --- URDF Writing ---
    prev_filename_handler = urdf._filename_handler
    if copy_mesh:
        new_urdf.write_xml_file(new_path)
        new_urdf._filename_handler = prev_filename_handler
    else:
        urdf._filename_handler = partial(yourdfpy.filename_handler_relative, dir=rel_dir)
        urdf.write_xml_file(new_path)
        urdf._filename_handler = prev_filename_handler