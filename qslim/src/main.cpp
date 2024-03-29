#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/per_face_normals.h>
//#include <igl/bfs_orient.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
#include <ctime>
//#include "collapse_callback.h"
#include "remove_duplicated.h"
#include "manifold.h"
//#include "preprocess.h"
#include "MeshSimplify.h"
#include <igl/decimate.h>
//#include "process.h"

#define INPUT_PATH ""
#define OUTPUT_PATH ""

int main(int argc, char * argv[])
{
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    //cout << "[filename.(off|obj|ply)]" << endl;
    //cout<<"  [space]  toggle animation."<<endl;
    //cout<<"  'r'  reset."<<endl;
    string input_filename(argv[1]);
    string output_filename(argv[2]);
    double ratio = std::stod(argv[3]);
    if (argc != 4) {
        cout << "Invalid Format : try to run './QEM <input_filename> <output_filename> <ratio of remain edge>'" << endl;
        return -1;
    }
    // Load mesh data
    MatrixXd OV;
    MatrixXi OF;
    read_triangle_mesh(INPUT_PATH + input_filename, OV, OF);

    qslim::MeshSimplify meshSimplify(OV, OF);
    meshSimplify.process(ratio);

    igl::writeOBJ(output_filename + ".obj", meshSimplify.get_vertices(), meshSimplify.get_faces());
/*    // Erase this for docker image
    igl::opengl::glfw::Viewer viewer;
    viewer.data().clear();
    //TODO: Update qValues using MeshSimplify class ; move callback function into meshSimplify class
    viewer.data().set_mesh(meshSimplify.get_vertices(), meshSimplify.get_faces());
    //viewer.data().set_mesh(VR, FR);
    viewer.data().set_face_based(true);
    return viewer.launch();*/

    return 0;
}