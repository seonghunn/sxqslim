//
// Created by seonghun on 7/18/23.
//

#include "collapse_callback.h"
using namespace std;

struct removed_vertices_index{
    int v1;
    int v2;
};

removed_vertices_index RV;

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
            // Your custom logic here. For example, you might want to prevent
            // collapsing edges that are on the boundary of the mesh:
            //
            // if (is_boundary_edge(e, V, F, E, EMAP, EF, EI)) {
            //     return false;
            // }
/*            if(!igl::is_edge_manifold(F)) {
                return false;
            }*/
            //cout << "pre_collapse : " << E << endl;
            RV.v1 = E(e,0);
            RV.v2 = E(e,1);

// Get new vertex index

            return true;  // Allow the edge to be collapsed.
        };

igl::decimate_post_collapse_callback post_collapse;

void setup_callbacks(QValues& qvalues) {
    post_collapse =
            [&qvalues](const Eigen::MatrixXd &V,
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

                // You can access and modify qvalues.values here.
                if(collapsed){
                    int RV_idx1 = RV.v1;
                    int RV_idx2 = RV.v2;
/*                    cout << "REMOVED VERTICES" << endl;
                    cout << RV.v1 << " " << RV.v2 << endl;
                    cout << endl;
                    std::cout << qvalues.values.size() << std::endl;
                    std::cout <<"new vertex "<< C.row(e) << std::endl;
                    //std::cout << "replacing v1 :" << V.row(E(e, 0)) << std::endl;
                    //std::cout << "replacing v1 index : " << v1 << std::endl;
                    //std::cout << "replacing v2 : " << V.row(E(e, 1)) << std::endl;
                    //std::cout << "replacing v2 index : " << v2 << std::endl;
                    std::cout << "collapsing edge : " << e << std::endl;
                    //std::cout << "new vertices : " << C << std::endl;
                    std::cout << "after decimation post collapse " << V << std::endl;
                    std::cout << std::endl;*/
                    Eigen::Matrix4d Q1 = qvalues.values[RV_idx1];
                    Eigen::Matrix4d Q2 = qvalues.values[RV_idx2];
                    qvalues.values[RV_idx1] = Q1 + Q2;
                    qvalues.values[RV_idx2] = Q1 + Q2;
                }
                // value change success
                //qvalues.values[0].row(0) << 1,1,1,1;

/*                if (collapsed) {
                    std::cout << "Edge " << e << " was collapsed." << std::endl;
                }*/
                //std::cout << "overall E" << E << std::endl;
            };
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
