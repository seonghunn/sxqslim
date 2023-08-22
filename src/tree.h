//
// Created by seonghun on 8/21/23.
//

#ifndef QEM_TREE_H
#define QEM_TREE_H

#include "AABB.hpp"
#include <Eigen/Core>


namespace qslim{
    //init AABB tree
    void initializeTreeFromMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, aabb::Tree &tree);
}


#endif //QEM_TREE_H
