#include "igl/opengl/glfw/Viewer.h"
#include "igl/decimate.h"
#include <iostream>
using namespace std;

int main(int argc, char *argv[])
{
/*  // Inline mesh of a cube
  const Eigen::MatrixXd V= (Eigen::MatrixXd(8,3)<<
    0.0,0.0,0.0,
    0.0,0.0,1.0,
    0.0,1.0,0.0,
    0.0,1.0,1.0,
    1.0,0.0,0.0,
    1.0,0.0,1.0,
    1.0,1.0,0.0,
    1.0,1.0,1.0).finished();

  const Eigen::MatrixXi F = (Eigen::MatrixXi(12,3)<<
    0,6,4,
    0,2,6,
    0,3,2,
    0,1,3,
    2,7,6,
    2,3,7,
    4,6,7,
    4,7,5,
    0,4,5,
    0,5,1,
    1,5,7,
    1,7,3).finished();*/
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    igl::readOBJ("../model/input/bunny.obj",V,F);

    cout << "Number of Input vertices : "<<V.rows()<<endl;
    cout << "Number of Input faces : "<<F.rows()<<endl;
    Eigen::MatrixXd V_out;
    Eigen::MatrixXi F_out;
    Eigen::VectorXi J;

  // Perform the decimation
    int target_num_faces = 1000;
    igl::decimate(V, F, target_num_faces, V_out, F_out, J);

    cout << "Number of Output vertices : "<<V_out.rows()<<endl;
    cout << "Number of Output faces : "<<F_out.rows()<<endl;

    igl::writeOBJ("../model/output/bunny_output.obj",V_out,F_out);

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V_out, F_out);
    viewer.data().set_face_based(true);
    viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
    viewer.launch();
}
