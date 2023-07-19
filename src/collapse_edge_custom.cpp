//
// Created by seonghun on 7/18/23.
//

#include "collapse_edge_custom.h"
#include <igl/collapse_edge.h>

bool collapse_edge_custom(
        const igl::decimate_cost_and_placement_callback & cost_and_placement,
        Eigen::MatrixXd & V,
        Eigen::MatrixXi & F,
        Eigen::MatrixXi & E,
        Eigen::VectorXi & EMAP,
        Eigen::MatrixXi & EF,
        Eigen::MatrixXi & EI,
        igl::min_heap< std::tuple<double,int,int> > & Q,
        Eigen::VectorXi & EQ,
        Eigen::MatrixXd & C
        )
{
    // Use qvalues here if necessary

    // Call the original function
    return igl::collapse_edge(cost_and_placement, V, F, E, EMAP, EF, EI, Q, EQ, C);
}