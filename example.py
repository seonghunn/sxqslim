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
mesh_simplify = QSlim.MeshSimplify(vertices, faces, 0.01, "output_filename")

# Access vertices and faces
vertices = mesh_simplify.get_vertices()
faces = mesh_simplify.get_faces()

# Perform the mesh simplification process
mesh_simplify.process()

# vertices and faces can now be used for further processing or visualization
