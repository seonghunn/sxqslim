import sys
sys.path.append('/Users/seonghun/Documents/study/research/sulab/code/MyCode/QEM-codebase/build')

import QSlim

import trimesh

# 파일 경로 설정
file_path = '/Users/seonghun/Documents/study/research/sulab/code/MyCode/QEM-codebase/model/input/bunny.obj'

# trimesh로 OBJ 파일 로드
mesh = trimesh.load(file_path)

# vertices와 faces 추출
vertices = mesh.vertices
faces = mesh.faces

# Initialize the MeshSimplify object with your parameters
mesh_simplify = QSlim.MeshSimplify(vertices, faces)

# Access vertices and faces
vertices = mesh_simplify.get_vertices()
faces = mesh_simplify.get_faces()

# Perform the mesh simplification process
verts, faces = mesh_simplify.process(ratio = 0.9)

simplified_mesh = trimesh.Trimesh(vertices=verts, faces=faces)

# 메시를 현재 경로에 OBJ 파일로 저장
output_file_path = './simplified_mesh.obj'
simplified_mesh.export(output_file_path)

print(f"Mesh saved to {output_file_path}")
# vertices and faces can now be used for further processing or visualization
