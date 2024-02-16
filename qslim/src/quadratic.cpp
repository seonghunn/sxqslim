//
// Created by seonghun on 7/18/23.
//

#include "quadratic.h"

using namespace Eigen;

namespace qslim{
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

        // Optimal
/*        Eigen::Vector4d target;
        if(A.determinant() > 1e-5){
            target = A.inverse() * Eigen::Vector4d(0, 0, 0, 1);
            p = target.head<3>() / target.w();
        }
        else{
            p = (V.row(v1) + V.row(v2)) / 2.0;
            target << p.transpose(), 1;
        }
        Eigen::MatrixXd result = target.transpose() * Q * target;
        cost = std::max(0.0, result(0, 0));*/

        // midpoint
        p = (V.row(v1) + V.row(v2)) / 2.0;
        Vector4d p_homogeneous;
        p_homogeneous << p.transpose(), 1;
        //cost = p_homogeneous.transpose() * Q * p_homogeneous;
        Eigen::MatrixXd result = p_homogeneous.transpose() * Q * p_homogeneous; // 이 연산의 결과는 1x1 행렬입니다.
        cost = std::max(0.0, result(0, 0));
    }
}
