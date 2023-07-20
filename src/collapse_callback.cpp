//
// Created by seonghun on 7/18/23.
//

#include "collapse_callback.h"
using namespace std;

namespace customCBF{
    struct removed_vertices_index{
        int v1;
        int v2;
    };

    removed_vertices_index RV;

// callback function for pre_collapse stage, this function always called every decimation step
    igl::decimate_pre_collapse_callback pre_collapse =
            [](const Eigen::MatrixXd &V,
               const Eigen::MatrixXi &F,
               const Eigen::MatrixXi &E,
               const Eigen::VectorXi &EMAP,
               const Eigen::MatrixXi &EF,
               const Eigen::MatrixXi &EI,
               const igl::min_heap< std::tuple<double,int,int> > &Q,
               const Eigen::VectorXi &EQ,
               const Eigen::MatrixXd &C,
               const int e) -> bool
            {
                // TODO: Your custom logic here. For example, you might want to prevent
                // collapsing edges that are on the boundary of the mesh:
                // if (is_boundary_edge(e, V, F, E, EMAP, EF, EI)) {
                //     return false;
                // }
                // collapsing edges that are not generate non-manifold mesh
/*            if(!igl::is_edge_manifold(F)) {
                return false;
            }*/

                // Get index of vertices which supposed to be replaced
                RV.v1 = E(e,0);
                RV.v2 = E(e,1);

                return true;  // Allow the edge to be collapsed.
            };
/*    igl::decimate_cost_and_placement_callback cost_and_placement;

    void setup_cost_and_placement_callback_with_qValues(QValues& qValues){
        cost_and_placement=
                [&qValues](
                        const int e,
                        const Eigen::MatrixXd & V,
                        const Eigen::MatrixXi & F,
                        const Eigen::MatrixXi & E,
                        const Eigen::VectorXi & *//*EMAP*//*,
                        const Eigen::MatrixXi & *//*EF*//*,
                        const Eigen::MatrixXi & *//*EI*//*,
                        double & cost,
                        Eigen::RowVectorXd & p)
                {
                    // E(e,0) returns the index of first vertex of edge e
                    int v1 = E(e, 0);
                    int v2 = E(e, 1);
                    Eigen::Matrix4d Q = qValues.values[v1] + qValues.values[v2];
                    Eigen::Matrix4d A;
                    A.row(0) << Q.row(0);
                    A.row(1) << Q.row(1);
                    A.row(2) << Q.row(2);
                    A.row(3) << 0, 0, 0, 1;

                    // new optimal point
                    //TODO: p is either vertices or midpoint if A is singular
                    Eigen::Vector4d target = A.inverse() * Eigen::Vector4d(0, 0, 0, 1);
                    // transform homogeneous coordinates to normal coordinates
                    p = target.head<3>() / target.w();

                    cost = target.transpose() * Q * target;
                    //cost = (V.row(E(e,0))-V.row(E(e,1))).norm();
                    //p = 0.5*(V.row(E(e,0))+V.row(E(e,1)));
                };
    }*/

// callback function for post_collapse stage, this function always called every decimation step
    igl::decimate_post_collapse_callback post_collapse;
// Wrapper function to use qValues table
    void setup_post_collapse_with_qValues(QValues& qValues) {
        post_collapse=
                [&qValues](const Eigen::MatrixXd &V,
                           const Eigen::MatrixXi &F,
                           const Eigen::MatrixXi &E,
                           const Eigen::VectorXi &EMAP,
                           const Eigen::MatrixXi &EF,
                           const Eigen::MatrixXi &EI,
                           const igl::min_heap< std::tuple<double,int,int> > &Q,
                           const Eigen::VectorXi &EQ,
                           const Eigen::MatrixXd &C,
                           const int e,
                           const int e1,
                           const int e2,
                           const int f1,
                           const int f2,
                           const bool collapsed)
                {
                    //Note : the resulting vertex after collapsing is C.row(e)
                    //Note : the vertices list V after collapsing changes to C (for each edge's end vertices)
                    // If edge is collapsed
                    if(collapsed){
                        int RV_idx1 = RV.v1;
                        int RV_idx2 = RV.v2;
                        // Update Q = Q1 + Q2 (index of Q might be indices of removed vertices)
                        Eigen::Matrix4d Q1 = qValues.values[RV_idx1];
                        Eigen::Matrix4d Q2 = qValues.values[RV_idx2];
                        qValues.values[RV_idx1] = Q1 + Q2;
                        qValues.values[RV_idx2] = Q1 + Q2;
                    }
                };
    }


    void quadratic(
            const int e,
            const Eigen::MatrixXd & V,
            const Eigen::MatrixXi & F,
            const Eigen::MatrixXi & E,
            const Eigen::VectorXi & /*EMAP*/,
            const Eigen::MatrixXi & /*EF*/,
            const Eigen::MatrixXi & /*EI*/,
            vector<Eigen::Matrix4d> & qValues,
            double & cost,
            Eigen::RowVectorXd & p)
    {
        // E(e,0) returns the index of first vertex of edge e
        int v1 = E(e, 0);
        int v2 = E(e, 1);
        Eigen::Matrix4d Q = qValues[v1] + qValues[v2];
        Eigen::Matrix4d A;
        A.row(0) << Q.row(0);
        A.row(1) << Q.row(1);
        A.row(2) << Q.row(2);
        A.row(3) << 0, 0, 0, 1;

        // new optimal point
        //TODO: p is either vertices or midpoint if A is singular
        Eigen::Vector4d target = A.inverse() * Eigen::Vector4d(0, 0, 0, 1);
        // transform homogeneous coordinates to normal coordinates
        p = target.head<3>() / target.w();

        cost = target.transpose() * Q * target;
        //cost = (V.row(E(e,0))-V.row(E(e,1))).norm();
//    p = 0.5*(V.row(E(e,0))+V.row(E(e,1)));
    }


/*
igl::decimate_stopping_condition_callback stopping_condition = [](
        const Eigen::MatrixXd& V,                          // V
        const Eigen::MatrixXi& F,                          // F
        const Eigen::MatrixXi& E,                          // E
        const Eigen::VectorXi& EMAP,                       // EMAP
        const Eigen::MatrixXi& EF,                         // EF
        const Eigen::MatrixXi& EI,                         // EI
        const igl::min_heap<std::tuple<double, int, int>>& Q,  // Q
        const Eigen::VectorXi& EQ,                         // EQ
        const Eigen::MatrixXd& C,                          // C
        const int e,                                       // e
        const int e1,                                      // e1
        const int e2,                                      // e2
        const int f1,                                      // f1
        const int f2                                       // f2
) {
    // Your code logic here
    // For example, you might want to stop if the number of faces is less than a certain threshold
    if (F.rows() < 30000) {
        return true;
    }

    // Or, you might want to stop if the cost of the next edge to collapse is above a certain threshold
*/
/*    if (!Q.empty() && std::get<0>(Q.top()) > 1.0) {
        return true;
    }*//*


    // Otherwise, continue the decimation process
    return false;
};
*/

}
