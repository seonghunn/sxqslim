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
namespace qslim{
    bool remove_duplicated_faces(MatrixXd& V, MatrixXi& F);
    // init Q value table using surface normal
    void reset(MatrixXd &V, MatrixXd &OV, MatrixXi &F, MatrixXi &OF,
               MatrixXi &E, VectorXi &EMAP, MatrixXi &EF, MatrixXi &EI, VectorXi &EQ,
               MatrixXd &C, igl::min_heap<std::tuple<double, int, int> > &Q, std::vector<Matrix4d> &cost_table,
               igl::opengl::glfw::Viewer &viewer, int &num_collapsed);

    // print MatrixXi, Xd
    template<typename Derived>
    void printMatrix(const Eigen::MatrixBase<Derived>& mat) {
        for (int i = 0; i < mat.rows(); i++) {
            cout<<mat.row(i)<<endl;
        }
        cout << endl;
    }
}

#endif //QEM_HELPER_H
