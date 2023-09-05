## QSlim implementation for greedy edge collapse
- using libigl, AABB tree (https://github.com/lohedges/aabbcc)
- add self intersection test using aabb tree for every iteration
- Input : Manifold, self-intersection free mesh, Output : Decimated Manifold, self-intersection free mesh

## Compile

Compile this project using the standard cmake routine:

    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O3" -DCMAKE_C_FLAGS_RELEASE="-O3"
    make

## Run

From within the `build` directory just issue:

    ./QSlim <input_filename in ./model/input> <output_filename> <ratio_of_collapsing>

If you set ratio as 0.3, only 30% of vertices will be remained.<br/>
A glfw app should launch displaying a 3D cube.

## Reference
Surface Simplification Using Quadric Error Metrics, 1997 </br>
https://www.cs.cmu.edu/~./garland/Papers/quadrics.pdf