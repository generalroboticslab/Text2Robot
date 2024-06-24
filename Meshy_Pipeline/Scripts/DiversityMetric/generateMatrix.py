import numpy as np
import ot 
from stl import mesh
from pyemd import emd


def load_stl(file_path):
    mesh_data = mesh.Mesh.from_file(file_path)
    vertices = mesh_data.vectors.reshape(-1, 3)
    return vertices

def emd_distance(vertices1, vertices2):
    M = ot.dist(vertices1, vertices2)  # Calculate cost matrix
    a = np.ones((vertices1.shape[0],)) / vertices1.shape[0]  # Uniform distribution for vertices1
    b = np.ones((vertices2.shape[0],)) / vertices2.shape[0]  # Uniform distribution for vertices2
    G0 = ot.emd(a, b, M)  # Compute EMD
    return (G0 * M).sum()


filepath1 = 'C:/Users/ringe/Spring2024Research/RobotsMakingRobots/Meshy_Pipeline/Scripts/DiversityMetric/Meshes/base_link1.stl'
filepath2 = 'C:/Users/ringe/Spring2024Research/RobotsMakingRobots/Meshy_Pipeline/Scripts/DiversityMetric/Meshes/base_link2.stl'

vertices1 = load_stl(filepath1)
vertices2 = load_stl(filepath2)

print(len(vertices1))

data = emd_distance(vertices1, vertices2)
print("Earth Mover's Distance:", data)