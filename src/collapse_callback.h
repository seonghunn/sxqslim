//
// Created by seonghun on 7/18/23.
//

#ifndef QEM_COLLAPSE_CALLBACK_H
#define QEM_COLLAPSE_CALLBACK_H

#include <igl/decimate_callback_types.h>
#include <igl/is_edge_manifold.h>
#include <igl/min_heap.h>
#include <igl/remove_unreferenced.h>
#include <igl/collapse_edge.h>
#include <Eigen/Core>
#include <vector>
#include "helper.h"
#include "manifold.h"

namespace qem{
    struct QValues {
        std::vector<Eigen::Matrix4d> values;
    };

    extern igl::decimate_pre_collapse_callback pre_collapse;
    extern igl::decimate_post_collapse_callback post_collapse;
    //extern igl::decimate_cost_and_placement_callback cost_and_placement;

    //void setup_cost_and_placement_with_qValues(QValues&);
    void setup_post_collapse_with_qValues(QValues&);
    void quadratic(
            const int e,
            const Eigen::MatrixXd & V,
            const Eigen::MatrixXi & /*F*/,
            const Eigen::MatrixXi & E,
            const Eigen::VectorXi & /*EMAP*/,
            const Eigen::MatrixXi & /*EF*/,
            const Eigen::MatrixXi & /*EI*/,
            std::vector<Eigen::Matrix4d> & qvalues,
            double & cost,
            Eigen::RowVectorXd & p);
}

#endif //QEM_COLLAPSE_CALLBACK_H