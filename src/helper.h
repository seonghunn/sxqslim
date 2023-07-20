//
// Created by seonghun on 7/20/23.
//

#ifndef QEM_HELPER_H
#define QEM_HELPER_H

#include <Eigen/Core>
#include <igl/remove_unreferenced.h>
#define IGL_COLLAPSE_EDGE_NULL 0

namespace customHPF{
    bool remove_duplicated_faces(Eigen::MatrixXd&, Eigen::MatrixXi&);
}


#endif //QEM_HELPER_H
