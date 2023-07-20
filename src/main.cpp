#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/per_face_normals.h>
#include <igl/remove_duplicate_vertices.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
#include "quadratic.h"
#include "collapse_callback.h"
#include "helper.h"

int main(int argc, char * argv[])
{
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    cout << "[filename.(off|obj|ply)]" << endl;
    cout<<"  [space]  toggle animation."<<endl;
    cout<<"  'r'  reset."<<endl;
    // Load mesh data
    string input_filename(argv[1]);
    string output_filename(argv[2]);
    if (argc != 3) {
        cout << "Invalid Format : try to run './QEM <input_filename> <output_filename>'" << endl;
        return -1;
        //input_filename = argv[1];
    }
    MatrixXd V,OV;
    MatrixXi F,OF;
    read_triangle_mesh(input_filename,OV,OF);
    // check whether mesh is manifold
    if(is_edge_manifold(OF)) cout << "Input model is Manifold mesh" << endl;
    else cout << "Input model is Non-Manifold mesh" << endl;
    //initialize viewer
    igl::opengl::glfw::Viewer viewer;
    // Prepare array-based edge data structures and priority queue
    VectorXi EMAP, EQ;
    MatrixXi E,EF,EI;
    igl::min_heap< std::tuple<double,int,int> > Q;
    // If an edge were collapsed, we'd collapse it to these points:
    MatrixXd C;
    int num_collapsed;

    // Surface normal per vertex to compute Q
    Eigen::MatrixXd N_faces;
    Eigen::MatrixXd N_homo(OF.rows(), 4);
    igl::per_face_normals(OV, OF, N_faces);

    // Transform to Homogeneous Coordinate
    for (int i = 0; i < N_faces.rows(); i++) {
        Eigen::RowVector3d n = N_faces.row(i);
        double d = -n.dot(OV.row(OF(i, 0)));
        N_homo.row(i) << n, d;
    }

    // Add initial Q value of each vertex to vector
    customCBF::QValues qValues;
    qValues.values.resize(OV.rows(), Eigen::Matrix4d::Zero());
    for (int i = 0; i < OF.rows(); i++) {
        Eigen::Vector4d p(N_homo(i, 0), N_homo(i, 1), N_homo(i, 2), N_homo(i, 3));
        Eigen::Matrix4d q = p * p.transpose();
        // Add q for each 3 vertex in face, addition for summing q for all adjacent planes
        for (int j = 0; j < 3; j++) {
            qValues.values[OF(i, j)] += q;
        }
    }

    // Wrapper function to use quadratic in collapse_edge_custom function
    auto quadratic_with_qValues = [&](const int e,
                                      const Eigen::MatrixXd & V,
                                      const Eigen::MatrixXi & F,
                                      const Eigen::MatrixXi & E,
                                      const Eigen::VectorXi & EMAP,
                                      const Eigen::MatrixXi & EF,
                                      const Eigen::MatrixXi & EI,
                                      double & cost,
                                      Eigen::RowVectorXd & p)
    {
        customCBF::quadratic(e, V, F, E, EMAP, EF, EI, qValues.values, cost, p);
    };
    // call wrap function to use qValues in callback function
    //customCBF::setup_cost_and_placement_with_qValues(qValues);
    customCBF::setup_post_collapse_with_qValues(qValues);

    // Function to reset original mesh and data structures
    const auto & reset = [&]()
    {
        F = OF;
        V = OV;
        edge_flaps(F,E,EMAP,EF,EI);
        C.resize(E.rows(),V.cols());
        VectorXd costs(E.rows());
        Q = {};
        EQ = Eigen::VectorXi::Zero(E.rows());
        {
            Eigen::VectorXd costs(E.rows());
            igl::parallel_for(E.rows(),[&](const int e)
            {
                double cost = e;
                RowVectorXd p(1,3);
                //reset each cost
                customCBF::quadratic(e, V, F, E, EMAP, EF, EI, qValues.values, cost, p);
                C.row(e) = p;
                costs(e) = cost;
            },10000);
            for(int e = 0;e<E.rows();e++)
            {
                Q.emplace(costs(e),e,0);
            }
        }

        num_collapsed = 0;
        viewer.data().clear();
        viewer.data().set_mesh(V,F);
        viewer.data().set_face_based(true);
    };

    const auto &pre_draw = [&](igl::opengl::glfw::Viewer & viewer)->bool
    {
        // If animating then collapse 10% of edges
        if(viewer.core().is_animating && !Q.empty())
        {
            // if stopping condition met, break
            if (num_collapsed>=10000) {
                viewer.core().is_animating = false;
                // remove duplicated vertices and faces
                Eigen::MatrixXi F2(F.rows(),3);
                Eigen::VectorXi J(F.rows());
                int m = 0;
                for(int f = 0;f<F.rows();f++)
                {
                    if(
                            F(f,0) != IGL_COLLAPSE_EDGE_NULL ||
                            F(f,1) != IGL_COLLAPSE_EDGE_NULL ||
                            F(f,2) != IGL_COLLAPSE_EDGE_NULL)
                    {
                        F2.row(m) = F.row(f);
                        J(m) = f;
                        m++;
                    }
                }
                F2.conservativeResize(m,F2.cols());
                J.conservativeResize(m);
                Eigen::VectorXi _1;
                Eigen::MatrixXd U;
                Eigen::MatrixXi G;
                Eigen::VectorXi I;
                igl::remove_unreferenced(V,F2,U,G,_1,I);
                V = U;
                F = G;
                if(is_edge_manifold(F)) cout << "Manifold mesh!" << endl;
                else cout << "Non-Manifold mesh!" << endl;
                return 0;
            }
            bool something_collapsed = false;
            // collapse edge
            const int max_iter = std::ceil(0.01*Q.size());
            for(int j = 0;j<max_iter;j++)
            {
                // if collapsing doesn't occur, break
                if(!collapse_edge(quadratic_with_qValues,
                                  customCBF::pre_collapse,
                                  customCBF::post_collapse,
                                  V,F,E,EMAP,EF,EI,Q,EQ,C))
                {
                    break;
                }
                something_collapsed = true;
                num_collapsed++;
            }

            if(something_collapsed)
            {
                if(is_edge_manifold(F)) cout << "Manifold mesh" << endl;
                else cout << "Non-Manifold mesh" << endl;
                //cout << V << endl;
                //cout << F << endl;
                viewer.data().clear();
                viewer.data().set_mesh(V,F);
                viewer.data().set_face_based(true);
            }
        }
        return false;
    };

    const auto &key_down =
            [&](igl::opengl::glfw::Viewer &viewer,unsigned char key,int mod)->bool
            {
                switch(key)
                {
                    case ' ': //space, stop
                        viewer.core().is_animating ^= 1;
                        //cout << F.size() << endl;
/*                        if (is_edge_manifold(F))cout << "Manifold Mesh" << endl;
                        else cout << "Non-Manifold Mesh" << endl;*/
                        break;
                    case 'R':
                    case 'r':
                        reset();
                        break;
                    default:
                        return false;
                }
                return true;
            };

    reset();
    viewer.core().background_color.setConstant(1);
    viewer.core().is_animating = true;
    viewer.callback_key_down = key_down;
    viewer.callback_pre_draw = pre_draw;
    return viewer.launch();
}
