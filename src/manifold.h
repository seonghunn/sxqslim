//
// Created by seonghun on 7/23/23.
//

#ifndef QEM_MANIFOLD_H
#define QEM_MANIFOLD_H
#include <Eigen/Core>
#include <igl/is_edge_manifold.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/polygon_mesh_processing/self_intersections.h>
#include <CGAL/Surface_mesh.h>

#include "../include/igl/fast_find_self_intersections.h"
#include "./AABB.hpp"
#include "tree.h"
#include <time.h>

using namespace Eigen;
using namespace std;
namespace qslim{
    bool check_mesh_orientation(const MatrixXd &V, const MatrixXi &F);
    bool check_self_intersection(const MatrixXd &V, const MatrixXi &F);
    bool is_manifold(const MatrixXd& V, const MatrixXi& F);
}


#endif //QEM_MANIFOLD_H
