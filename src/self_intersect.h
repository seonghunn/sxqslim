//
// Created by seonghun on 8/23/23.
//

#ifndef QSLIM_SELF_INTERSECT_H
#define QSLIM_SELF_INTERSECT_H

#include <Eigen/Core>
#include <iostream>
#include <igl/intersect.h>
#include <../include/tri_tri_intersect.h>
#include "AABB.hpp"

using namespace Eigen;
using namespace std;
namespace qslim{
    // exact triangle intersection test at 3d dim
    bool tri_tri_intersection_check(const MatrixXd &V, const MatrixXi &F, unsigned int faceIdx1, unsigned int faceIdx2);

    // self intersection test, return true if there is self intersection
    bool self_intersection_test(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                                unordered_map<int, bool> &decimated_faces);
}


#endif //QSLIM_SELF_INTERSECT_H
