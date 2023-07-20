#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/per_face_normals.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
#include "quadratic.h"
#include "collapse_edge_custom.h"
#include "collapse_callback.h"

int main(int argc, char * argv[])
{
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    cout<<"Usage: ./703_Decimation_bin [filename.(off|obj|ply)]"<<endl;
    cout<<"  [space]  toggle animation."<<endl;
    cout<<"  'r'  reset."<<endl;
    // Load a closed manifold mesh
    string filename(argv[0]);
    if(argc>=2)
    {
        filename = argv[1];
    }
    MatrixXd V,OV;
    MatrixXi F,OF;
    read_triangle_mesh(filename,OV,OF);

    igl::opengl::glfw::Viewer viewer;

    // Prepare array-based edge data structures and priority queue
    VectorXi EMAP;
    MatrixXi E,EF,EI;
    igl::min_heap< std::tuple<double,int,int> > Q;
    Eigen::VectorXi EQ;
    // If an edge were collapsed, we'd collapse it to these points:
    MatrixXd C;

    int num_collapsed;

    // Surface normal per vertex
    Eigen::MatrixXd N_faces;
    Eigen::MatrixXd N_homo(OF.rows(), 4);
    igl::per_face_normals(OV, OF, N_faces);

    // Homogeneous Coordinate
    for (int i = 0; i < N_faces.rows(); i++) {
        Eigen::RowVector3d n = N_faces.row(i);
        double d = -n.dot(OV.row(OF(i, 0)));
        N_homo.row(i) << n, d;
    }

    // Add initial Q value to vector
    QValues qvalues;
    qvalues.values.resize(OV.rows(), Eigen::Matrix4d::Zero());
    for (int i = 0; i < OF.rows(); i++) {
        Eigen::Vector4d p(N_homo(i, 0), N_homo(i, 1), N_homo(i, 2), N_homo(i, 3));
        Eigen::Matrix4d q = p * p.transpose();
        // Add q for each 3 vertex in face, addition for summing q for all adjacent planes
        for (int j = 0; j < 3; j++) {
            qvalues.values[OF(i,j)] += q;
        }
    }

    // Lambda function to use quadratic in collapse_edge_custom function
    auto quadratic_with_qvalues = [&](const int e,
                                      const Eigen::MatrixXd & V,
                                      const Eigen::MatrixXi & F,
                                      const Eigen::MatrixXi & E,
                                      const Eigen::VectorXi & EMAP,
                                      const Eigen::MatrixXi & EF,
                                      const Eigen::MatrixXi & EI,
                                      double & cost,
                                      Eigen::RowVectorXd & p)
    {
        quadratic(e, V, F, E, EMAP, EF, EI, qvalues.values, cost, p);
    };
    // call wrap function to use qvalues in callback function
    setup_callbacks(qvalues);


    // Function to reset original mesh and data structures
    const auto & reset = [&]()
    {
        F = OF;
        V = OV;
        edge_flaps(F,E,EMAP,EF,EI);
        C.resize(E.rows(),V.cols());
        VectorXd costs(E.rows());
        // https://stackoverflow.com/questions/2852140/priority-queue-clear-method
        // Q.clear();
        Q = {};
        EQ = Eigen::VectorXi::Zero(E.rows());
        {
            Eigen::VectorXd costs(E.rows());
            igl::parallel_for(E.rows(),[&](const int e)
            {
                double cost = e;
                RowVectorXd p(1,3);
                quadratic(e,V,F,E,EMAP,EF,EI,qvalues.values,cost,p);
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
            bool something_collapsed = false;
            // collapse edge
            const int max_iter = std::ceil(0.01*Q.size());
            for(int j = 0;j<max_iter;j++)
            {
                if(!collapse_edge(quadratic_with_qvalues,pre_collapse,post_collapse,V,F,E,EMAP,EF,EI,Q,EQ,C))
                {
                    break;
                }
                something_collapsed = true;
                num_collapsed++;
            }

            if(something_collapsed)
            {
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
                    case ' ':
                        viewer.core().is_animating ^= 1;
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
