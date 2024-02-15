import sys
sys.path.append('/Users/seonghun/Documents/study/research/sulab/code/MyCode/QEM-codebase/build')

import qslim_module

# Initialize the MeshSimplify object with your parameters
mesh_simplify = qslim_module.MeshSimplify(vertices, faces, ratio, output_filename)

# Access vertices and faces
vertices = mesh_simplify.get_vertices()
faces = mesh_simplify.get_faces()

# Perform the mesh simplification process
mesh_simplify.process()

# vertices and faces can now be used for further processing or visualization
