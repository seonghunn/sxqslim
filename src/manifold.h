//
// Created by seonghun on 7/23/23.
//

#ifndef QEM_MANIFOLD_H
#define QEM_MANIFOLD_H
#include <Eigen/Core>
#include <igl/is_edge_manifold.h>

using namespace Eigen;
using namespace std;
namespace qem{
    bool check_mesh_orientation(const MatrixXd &V, const MatrixXi &F);
    bool check_self_intersection(const MatrixXd &V, const MatrixXi &F);
    bool is_manifold(const MatrixXd& V, const MatrixXi& F);
}


#endif //QEM_MANIFOLD_H
