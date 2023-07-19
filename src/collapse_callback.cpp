//
// Created by seonghun on 7/18/23.
//

#include "collapse_callback.h"


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
            if(!igl::is_edge_manifold(F)) {
                return false;
            }

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
                // You can access and modify qvalues.values here.
                //std::cout << qvalues.values[0] << std::endl;
                //std::cout <<"new vertex "<< C.row(e) << std::endl;
                // value change success
                //qvalues.values[0].row(0) << 1,1,1,1;

/*                if (collapsed) {
                    std::cout << "Edge " << e << " was collapsed." << std::endl;
                }*/
                std::cout << "overall E" << E << std::endl;
            };
}