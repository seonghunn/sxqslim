//
// Created by seonghun on 7/18/23.
//

#include "quadratic.h"

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
        Eigen::RowVectorXd & p)
{

    cout << "q" << qvalues[0] << endl;
    qvalues[0].row(0) << 0,0,0,0;
    cost = (V.row(E(e,0))-V.row(E(e,1))).norm();
    p = 0.5*(V.row(E(e,0))+V.row(E(e,1)));
}