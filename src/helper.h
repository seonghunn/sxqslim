//
// Created by seonghun on 7/20/23.
//

#ifndef QEM_HELPER_H
#define QEM_HELPER_H

#include <Eigen/Core>
#include <igl/remove_unreferenced.h>
#include <igl/edge_flaps.h>
#include <igl/parallel_for.h>
#include <igl/opengl/glfw/Viewer.h>
#include "collapse_callback.h"
#define IGL_COLLAPSE_EDGE_NULL 0

using namespace Eigen;
namespace customHPF{
    bool remove_duplicated_faces(MatrixXd& V, MatrixXi& F);
    void reset(MatrixXd &V, MatrixXd &OV, MatrixXi &F, MatrixXi &OF,
               MatrixXi &E, VectorXi &EMAP, MatrixXi &EF, MatrixXi &EI, VectorXi &EQ,
               MatrixXd &C, igl::min_heap<std::tuple<double, int, int> > &Q, std::vector<Matrix4d> &cost_table,
               igl::opengl::glfw::Viewer &viewer, int &num_collapsed);
}

#endif //QEM_HELPER_H
