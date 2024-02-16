//
// Created by seonghun on 7/20/23.
//

#include "remove_duplicated.h"

using namespace std;
namespace qslim {
    bool remove_duplicated_faces(Eigen::MatrixXd &V, Eigen::MatrixXi &F) {
        Eigen::MatrixXi F2(F.rows(), 3);
        Eigen::VectorXi J(F.rows());
        int m = 0;
        for (int f = 0; f < F.rows(); f++) {
            if (
                    F(f, 0) != IGL_COLLAPSE_EDGE_NULL ||
                    F(f, 1) != IGL_COLLAPSE_EDGE_NULL ||
                    F(f, 2) != IGL_COLLAPSE_EDGE_NULL) {
                F2.row(m) = F.row(f);
                J(m) = f;
                m++;
            }
        }
        F2.conservativeResize(m, F2.cols());
        J.conservativeResize(m);
        Eigen::VectorXi _1;
        Eigen::MatrixXd U;
        Eigen::MatrixXi G;
        Eigen::VectorXi I;
        igl::remove_unreferenced(V, F2, U, G, _1, I);
        V = U;
        F = G;
        //TODO: exception handling

        return true;
    }
}