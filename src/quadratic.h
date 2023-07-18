//
// Created by seonghun on 7/18/23.
//

#ifndef QEM_QUADRATIC_H
#define QEM_QUADRATIC_H

#include "Eigen/Core"
void quadratic(
        const int e,
        const Eigen::MatrixXd & V,
        const Eigen::MatrixXi & /*F*/,
        const Eigen::MatrixXi & E,
        const Eigen::VectorXi & /*EMAP*/,
        const Eigen::MatrixXi & /*EF*/,
        const Eigen::MatrixXi & /*EI*/,
        double & cost,
        Eigen::RowVectorXd & p);

#endif //QEM_QUADRATIC_H
