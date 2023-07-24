//
// Created by seonghun on 7/23/23.
//

#include "process.h"

namespace qem{
    bool process(const decimate_cost_and_placement_callback &cost_and_placement,
                 const decimate_pre_collapse_callback &pre_collapse,
                 const decimate_post_collapse_callback &post_collapse,
                 Eigen::MatrixXd &V,
                 Eigen::MatrixXi &F,
                 Eigen::MatrixXi &E,
                 Eigen::VectorXi &EMAP,
                 Eigen::MatrixXi &EF,
                 Eigen::MatrixXi &EI,
                 VectorXi &EQ,
                 MatrixXd &C,
                 min_heap<std::tuple<double, int, int> > &Q,
                 int stopping_condition,
                 int& num_collapsed
    ) {
        {
            bool flag = false;
            // If animating then collapse 10% of edges
            while (!Q.empty()) {
                if (flag) break;
                cout << "process start" << endl;
                bool something_collapsed = false;
                // collapse edge
                const int max_iter = std::ceil(0.01 * Q.size());
                for (int j = 0; j < max_iter; j++) {
                    // if collapsing doesn't occur, break
                    if (!collapse_edge(cost_and_placement,
                                       pre_collapse,
                                       post_collapse,
                                       V, F, E, EMAP, EF, EI, Q, EQ, C)) {
                        break;
                    }
                    something_collapsed = true;
                    num_collapsed++;
                    // if stopping condition met, break
                    if (num_collapsed >= stopping_condition) {
                        flag = true;
                        // remove duplicated vertices and faces
                        qem::remove_duplicated_faces(V, F);
                        cout << "\n" << "*******************************" << endl;
                        if (is_edge_manifold(F)) cout << "Resulting mesh is Manifold" << endl;
                        else cout << "Resulting mesh is Non-Manifold" << endl;
                        cout << "*******************************" << endl;
                        break;
                    }
                }

                if (something_collapsed) {
                    cout << num_collapsed << " vertices are removed" << endl;
/*                    if (flag) {
                        if (writeOBJ(OUTPUT_PATH + output_filename + ".obj", V, F)) {
                            cout << "Successfully wrote to " << output_filename << ".obj" << endl;
                            return 0;
                        } else {
                            cout << "Failed to wrote to " << output_filename << ".obj" << endl;
                            return -1;
                        }
                    }*/
                }
            }
            return false;
        };
    }
}