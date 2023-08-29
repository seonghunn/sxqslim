//
// Created by seonghun on 7/20/23.
//

#ifndef QEM_HELPER_H
#define QEM_HELPER_H

#include <Eigen/Core>
#include <igl/remove_unreferenced.h>
#include <iostream>
#define IGL_COLLAPSE_EDGE_NULL 0

using namespace std;
using namespace Eigen;
namespace qslim{
    struct index_of_removed_vertices{
        int v1;
        int v2;
    };

    // remove duplicated faces at V and F
    bool remove_duplicated_faces(MatrixXd& V, MatrixXi& F);
}

#endif //QEM_HELPER_H
