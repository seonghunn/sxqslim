//
// Created by seonghun on 8/23/23.
//

#ifndef QSLIM_SELF_INTERSECT_H
#define QSLIM_SELF_INTERSECT_H

#include <Eigen/Core>
#include <iostream>
#include <igl/intersect.h>
#include <unordered_set>
//#include <../include/tri_tri_intersect.h>
#include "AABB.hpp"
#include <igl/circulation.h>

using namespace Eigen;
using namespace std;
namespace qslim{
    // Is two aabb intersect?
    bool aabb_intersecting(const aabb::AABB &a, const aabb::AABB &b);

    // set aabb
    aabb::AABB set_current_aabb(const MatrixXd &V, const MatrixXi &F, int faceIdx);

    // traverse AABB Tree and get candidates
    void dfs(aabb::Tree &tree, int nodeId, aabb::AABB &queryAABB, vector<int> &candidates);

    // triangle intersection check for intersection candidates
    bool tri_tri_intersection_check(const MatrixXd &V, const MatrixXi &F, unsigned int faceIdx1, unsigned int faceIdx2,
                                    int shared_vertex);

    // exact self intersection check
    /*!
     * \return true if there are self intersections
     */
    bool self_intersection_check_full(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                                 unordered_map<int, bool> &decimated_faces);


    void get_two_ring_neigh(MatrixXi &F,
                            unordered_map<int, vector<int>> &affected_triangle_list,
                            vector<int> &combined_affected_triangle_indices,
                            vector<int> &two_ring_neigh);

    void get_one_ring_neigh(
            const int e,
            const Eigen::MatrixXi &F,
            const Eigen::VectorXi &EMAP,
            const Eigen::MatrixXi &EF,
            const Eigen::MatrixXi &EI,
            std::vector<int> &one_ring_faces);
    // self intersection check for iteration (pre_collapse_callback)
    bool self_intersection_check_local(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                                 unordered_map<int, bool> &decimated_faces,
                                 vector<int> &affected_triangle_indices, int removed_vertex_idx1,
                                 int removed_vertex_idx2);
}


#endif //QSLIM_SELF_INTERSECT_H
