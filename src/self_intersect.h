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
    // Is two aabb intersect?
    bool aabb_intersecting(const aabb::AABB &a, const aabb::AABB &b);

    // set aabb
    aabb::AABB set_current_aabb(const MatrixXd &V, const MatrixXi &F, int faceIdx);

    // traverse AABB Tree and get candidates
    void dfs(aabb::Tree &tree, int nodeId, aabb::AABB &queryAABB, vector<int> &candidates);

    // triangle intersection check for candidates using AABB
    bool tri_tri_intersection_check(const MatrixXd &V, const MatrixXi &F, unsigned int faceIdx1, unsigned int faceIdx2);

    // exact self intersection check
    /*!
     * \return true if there are self intersections
     */
    bool self_intersection_check(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                                 unordered_map<int, bool> &decimated_faces);
/*    // exact triangle intersection test at 3d dim
    bool tri_tri_intersection_check(const MatrixXd &V, const MatrixXi &F, unsigned int faceIdx1, unsigned int faceIdx2);

    // self intersection test, return true if there is self intersection
    bool self_intersection_test(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                                unordered_map<int, bool> &decimated_faces);*/
}


#endif //QSLIM_SELF_INTERSECT_H
