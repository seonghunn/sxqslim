//
// Created by seonghun on 7/23/23.
//

#ifndef QEM_PROCESS_H
#define QEM_PROCESS_H

#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/decimate_callback_types.h>
#include <igl/is_edge_manifold.h>
#include <igl/collapse_edge.h>
#include "helper.h"
#include "collapse_callback.h"

using namespace std;
using namespace Eigen;
using namespace igl;
namespace qslim{
    bool process(const decimate_cost_and_placement_callback &cost_and_placement,
                 const decimate_pre_collapse_callback &pre_collapse,
                 const decimate_post_collapse_callback &post_collapse,
                 Eigen::MatrixXd &V,
                 Eigen::MatrixXi &F,
                 Eigen::MatrixXi &E,
                 Eigen::VectorXi &EMAP,
                 Eigen::MatrixXi &EF,
                 Eigen::MatrixXi &EI,
                 VectorXi &EQ,
                 MatrixXd &C,
                 min_heap<std::tuple<double, int, int> > &Q,
                 int stopping_condition,
                 int& num_collapsed
    );
}


#endif //QEM_PROCESS_H
