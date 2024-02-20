import sxqslim
import trimesh

file_path = './mesh/bunny.obj'

mesh = trimesh.load(file_path)
vertices = mesh.vertices
faces = mesh.faces

# Initialize the MeshSimplify object with your parameters
mesh_simplify = sxqslim.MeshSimplify(vertices, faces)

# Perform the mesh simplification process
verts, faces = mesh_simplify.process(ratio = 0.01)

simplified_mesh = trimesh.Trimesh(vertices=verts, faces=faces)

output_file_path = './simplified_mesh.obj'
simplified_mesh.export(output_file_path)

print(f"Mesh saved to {output_file_path}")
