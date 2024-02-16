//
// Created by seonghun on 7/18/23.
//

#ifndef QEM_QUADRATIC_H
#define QEM_QUADRATIC_H

#include "Eigen/Core"
#include <igl/per_vertex_normals.h>
#include <vector>
#include <algorithm>
using namespace std;

namespace qslim{
    /*!
     *
     * @param e
     * @param V
     * @param E
     * @param qvalues qValues table
     * @param cost cost for min-heap
     * @param p corresponding optimal position
     */
    void quadratic(
            const int e,
            const Eigen::MatrixXd & V,
            const Eigen::MatrixXi & /*F*/,
            const Eigen::MatrixXi & E,
            const Eigen::VectorXi & /*EMAP*/,
            const Eigen::MatrixXi & /*EF*/,
            const Eigen::MatrixXi & /*EI*/,
            vector<Eigen::Matrix4d> & qvalues,
            double & cost,
            Eigen::RowVectorXd & p);
}

#endif //QEM_QUADRATIC_H
