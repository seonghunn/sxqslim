//
// Created by seonghun on 8/21/23.
//

#ifndef QEM_TREE_H
#define QEM_TREE_H

#include "AABB.hpp"
#include "helper.h"
#include <Eigen/Core>

using namespace Eigen;
namespace qslim{
    //init AABB tree
    void initialize_tree_from_mesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, aabb::Tree &tree);

    //update tree after decimation
    void update_tree_after_decimation(
            const Eigen::MatrixXd &V,
            const Eigen::MatrixXi &F,
            aabb::Tree &tree,
            int RV_idx1, int RV_idx2,
            int f1, int f2);

    //update ancestors of leaf
    void update_ancestors();
}


#endif //QEM_TREE_H
