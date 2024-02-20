## sxqslim : Self-intersection free mesh simplification with QEM
`sxqslim` is a Python package for mesh simplification that ensures the output mesh is manifold and free of self-intersections. It leverages **Quadric Error Metrics (QEM)** for simplification and employs an **AABB tree** for efficient self-intersection detection during each iteration.

![img.png](img/tyra.png) ![img_1.png](img/tyra_collapsed.png)
- Utilizes `libigl` and an AABB tree for spatial indexing (https://github.com/lohedges/aabbcc)
- Adds self-intersection tests using the AABB tree for every iteration
- `input`: **Manifold, self-intersection free mesh**
- `Output`: **Decimated manifold, self-intersection free mesh**
## Compile

```angular2html
cd qslim
git submodule add https://github.com/libigl/libigl.git external/libigl
git submodule update --init --recursive
pip install -e .
```

## Simplifying a Mesh
1. Load your mesh using a compatible mesh loader, such as `trimesh`.
2. Initialize the `MeshSimplify` object with the `vertices` and `faces` of your mesh. This step initializes various data structures required for the simplification process, including the AABB tree for spatial indexing and edge data for connectivity analysis.
3. Call the `process` method on the `MeshSimplify` object, specifying the simplification ratio.
4. Use the simplified vertices and faces to create a new mesh.
### Example code
```
import sxqslim
import trimesh

file_path = './mesh/bunny.obj'

mesh = trimesh.load(file_path)
vertices = mesh.vertices
faces = mesh.faces

# Initialize the MeshSimplify object
mesh_simplify = sxqslim.MeshSimplify(vertices, faces)

# Perform the mesh simplification process
verts, faces = mesh_simplify.process(ratio=0.01)

simplified_mesh = trimesh.Trimesh(vertices=verts, faces=faces)

output_file_path = './simplified_mesh.obj'
simplified_mesh.export(output_file_path)

print(f"Mesh saved to {output_file_path}")
```
Run the example code with:
```
python example.py
```
Setting the `ratio` to 0.3 retains only 30% of the original vertices.

## Reference
Surface Simplification Using Quadric Error Metrics, 1997 </br>
https://www.cs.cmu.edu/~./garland/Papers/quadrics.pdf