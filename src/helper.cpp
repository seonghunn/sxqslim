//
// Created by seonghun on 7/20/23.
//

#include "helper.h"
#include <igl/AABB.h>

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

    // set all initial value before iteration, compute each cost, position and push into initial priority queue
    void init_queue(MatrixXd &V, MatrixXd &OV, MatrixXi &F, MatrixXi &OF,
                    MatrixXi &E, VectorXi &EMAP, MatrixXi &EF, MatrixXi &EI, VectorXi &EQ,
                    MatrixXd &C, igl::min_heap<std::tuple<double, int, int> > &Q, std::vector<Matrix4d> &cost_table,
                    igl::opengl::glfw::Viewer &viewer, int &num_collapsed) {

        F = OF;
        V = OV;
        igl::edge_flaps(F, E, EMAP, EF, EI);
        C.resize(E.rows(), V.cols());
        VectorXd costs(E.rows());
        Q = {};
        EQ = Eigen::VectorXi::Zero(E.rows());
        {
            Eigen::VectorXd costs(E.rows());
            //parallel code
            igl::parallel_for(E.rows(), [&](const int e) {
                double cost = e;
                RowVectorXd p(1, 3);
                //reset each cost
                qslim::quadratic(e, V, F, E, EMAP, EF, EI, cost_table, cost, p);
                C.row(e) = p;
                costs(e) = cost;
            }, 10000);
            for (int e = 0; e < E.rows(); e++) {
                Q.emplace(costs(e), e, 0);
            }

            //serialize
/*
            for (int e = 0; e < E.rows(); e++) {
                double cost = e;
                RowVectorXd p(3);
                //reset each cost
                qslim::quadratic(e, V, F, E, EMAP, EF, EI, cost_table, cost, p);
                C.row(e) = p;
                costs(e) = cost;
            }

            for (int e = 0; e < E.rows(); e++) {
                Q.emplace(costs(e), e, 0);
            }
*/

        }

        num_collapsed = 0;
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        viewer.data().set_face_based(true);
    }
    template<typename Derived>
    void printMatrix(const Eigen::MatrixBase<Derived>& mat);
}