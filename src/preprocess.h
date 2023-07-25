//
// Created by seonghun on 7/24/23.
//

#ifndef QEM_PREPROCESS_H
#define QEM_PREPROCESS_H

#include <Eigen/Core>
#include <igl/per_face_normals.h>
#include "manifold.h"

using namespace Eigen;
using namespace std;
namespace qem{
    // set orientation of input mesh goes outward
    void set_input_orient_outward(MatrixXd &V, MatrixXi &F, MatrixXi &FF);

    // get homogeneous surface normal per faces
    void get_normal_homo_per_face(MatrixXd &V, MatrixXi &F, MatrixXd &N_homo);

    // initialize qValues table
    void init_qValues(MatrixXd &V, MatrixXi &F, MatrixXd &N_homo, std::vector<Matrix4d> &table);
}

#endif //QEM_PREPROCESS_H
