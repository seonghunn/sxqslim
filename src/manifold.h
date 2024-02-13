//
// Created by seonghun on 7/23/23.
//

#ifndef QEM_MANIFOLD_H
#define QEM_MANIFOLD_H
#include <Eigen/Core>
#include <igl/is_edge_manifold.h>

//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/polygon_mesh_processing/self_intersections.h>
//#include <CGAL/Surface_mesh.h>

#include "../include/igl/fast_find_self_intersections.h"
#include "../include/AABB.hpp"
#include "tree.h"
#include <time.h>
#include "self_intersect.h"

using namespace Eigen;
using namespace std;
namespace qslim{
    bool check_edge_manifold(const MatrixXd &V, const MatrixXi &F);

    bool check_mesh_orientation(const MatrixXd &V, const MatrixXi &F);

    // self intersection for input and output test
    bool check_self_intersection(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                                 unordered_map<int, bool> &decimated_faces);

    // self intersection check for iteration (pre_collapse_callback)
    bool check_self_intersection(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                     unordered_map<int, bool> &decimated_faces,
                     vector<int> &affected_triangle_indices, int removed_vertex_idx1,
                     int removed_vertex_idx2);

    // manifold test for input and output
    bool is_manifold(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                     unordered_map<int, bool> &decimated_faces, bool useManifoldCheck);

    // manifold check for iteration (pre_collapse_callback)
    bool is_manifold(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                     unordered_map<int, bool> &decimated_faces,
                     vector<int> &affected_triangle_indices, int removed_vertex_idx1,
                     int removed_vertex_idx2, bool useManifoldCheck);

}


#endif //QEM_MANIFOLD_H
