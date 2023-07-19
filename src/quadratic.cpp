//
// Created by seonghun on 7/18/23.
//

#include "quadratic.h"

void quadratic(
        const int e,
        const Eigen::MatrixXd & V,
        const Eigen::MatrixXi & F,
        const Eigen::MatrixXi & E,
        const Eigen::VectorXi & /*EMAP*/,
        const Eigen::MatrixXi & /*EF*/,
        const Eigen::MatrixXi & /*EI*/,
        vector<Eigen::Matrix4d> & qvalues,
        double & cost,
        Eigen::RowVectorXd & p)
{
    // E(e,0) returns the index of first vertex of edge e
    int v1 = E(e, 0);
    int v2 = E(e, 1);
    Eigen::Matrix4d Q = qvalues[v1] + qvalues[v2];
    Eigen::Matrix4d A;
    A.row(0) << Q.row(0);
    A.row(1) << Q.row(1);
    A.row(2) << Q.row(2);
    A.row(3) << 0, 0, 0, 1;

    // new optimal point
    Eigen::Vector4d target = A.inverse() * Eigen::Vector4d(0, 0, 0, 1);
    // transform homogeneous coordinates to normal coordinates
    p = target.head<3>() / target.w();

    cost = target.transpose() * Q * target;
    //cost = (V.row(E(e,0))-V.row(E(e,1))).norm();
//    p = 0.5*(V.row(E(e,0))+V.row(E(e,1)));
}