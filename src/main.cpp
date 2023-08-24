#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/per_face_normals.h>
//#include <igl/bfs_orient.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
#include <ctime>
#include "collapse_callback.h"
#include "helper.h"
#include "manifold.h"
#include "preprocess.h"
#include "MeshSimplify.h"
//#include "process.h"

#define INPUT_PATH "../model/input/"
#define OUTPUT_PATH "../model/output/"

int main(int argc, char * argv[])
{
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    cout << "[filename.(off|obj|ply)]" << endl;
    cout<<"  [space]  toggle animation."<<endl;
    //cout<<"  'r'  reset."<<endl;
    string input_filename(argv[1]);
    string output_filename(argv[2]);
    double ratio = std::stod(argv[3]);
    if (argc != 4) {
        cout << "Invalid Format : try to run './QEM <input_filename> <output_filename> <ratio of remain edge>'" << endl;
        return -1;
    }
    // Load mesh data
    MatrixXd V, V_, OV;
    MatrixXi F, F_, OF;
    //read_triangle_mesh(INPUT_PATH + input_filename, OV, F_);
    read_triangle_mesh(INPUT_PATH + input_filename, OV, OF);
    // set orient outward
/*    qslim::set_input_orient_outward(OV, F_, OF);
    for (int i = 0; i < OF.rows(); i++) {
        cout << "F_ : " << F_.row(i) << " OF : " << OF.row(i) << endl;
    }*/
    // check whether mesh is manifold
    //if(is_edge_manifold(OF)) {

    //TODO: Put this into initializer at MeshSimplify class
/*    if(qslim::is_manifold(OV, OF)){
        cout << "\n" << "*******************************" << endl;
        cout << "Input model is Manifold mesh" << endl;
        cout << "Number of Vertex : " << OV.rows() << endl;
        cout << "Number of Faces : " << OF.rows() << endl;
        cout << "*******************************" << "\n" << endl;
    }
    else {
        cout << "Input model is Non-Manifold mesh" << endl;
        cout << "Please use Manifold mesh" << endl;
        return -1;
    }*/
    igl::opengl::glfw::Viewer viewer;
    qslim::MeshSimplify meshSimplify(OV, OF, ratio);
    meshSimplify.process();

    // Erase this for docker image
    viewer.data().clear();
    //TODO: Update qValues using MeshSimplify class ; move callback function into meshSimplify class
    viewer.data().set_mesh(meshSimplify.get_vertices(), meshSimplify.get_faces());
    //viewer.data().set_mesh(V, F);
    viewer.data().set_face_based(true);
    return viewer.launch();

    //return 0;
}