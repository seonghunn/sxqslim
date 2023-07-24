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
#include <igl/is_edge_manifold.h>
#include <igl/facet_components.h>
#include <igl/orient_outward.h>
//#include <igl/copyleft/cgal/RemeshSelfIntersectionsParam.h>
//#include <igl/copyleft/cgal/remesh_self_intersections.h>
//#include <igl/orientable.h>

#include "manifold.h"
#include "collapse_callback.h"
#define IGL_COLLAPSE_EDGE_NULL 0

using namespace Eigen;
namespace qem{
    bool remove_duplicated_faces(MatrixXd& V, MatrixXi& F);
    void reset(MatrixXd &V, MatrixXd &OV, MatrixXi &F, MatrixXi &OF,
               MatrixXi &E, VectorXi &EMAP, MatrixXi &EF, MatrixXi &EI, VectorXi &EQ,
               MatrixXd &C, igl::min_heap<std::tuple<double, int, int> > &Q, std::vector<Matrix4d> &cost_table,
               igl::opengl::glfw::Viewer &viewer, int &num_collapsed);
    void set_input_orient_outward(MatrixXd &V, MatrixXi &F, MatrixXi &FF);
}

#endif //QEM_HELPER_H
