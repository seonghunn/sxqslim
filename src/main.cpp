#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/per_face_normals.h>
//#include <igl/bfs_orient.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
#include <ctime>
#include "collapse_callback.h"
#include "helper.h"
#include "manifold.h"
#include "preprocess.h"
//#include "process.h"

#define INPUT_PATH "../model/input/"
#define OUTPUT_PATH "../model/output/"

int main(int argc, char * argv[])
{
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    cout << "[filename.(off|obj|ply)]" << endl;
    cout<<"  [space]  toggle animation."<<endl;
    //cout<<"  'r'  reset."<<endl;
    string input_filename(argv[1]);
    string output_filename(argv[2]);
    double ratio = std::stod(argv[3]);
    if (argc != 4) {
        cout << "Invalid Format : try to run './QEM <input_filename> <output_filename> <ratio of remain edge>'" << endl;
        return -1;
    }
    // Load mesh data
    MatrixXd V, V_, OV;
    MatrixXi F, F_, OF;
    //read_triangle_mesh(INPUT_PATH + input_filename, OV, F_);
    read_triangle_mesh(INPUT_PATH + input_filename, OV, OF);
    // set orient outward
/*    qem::set_input_orient_outward(OV, F_, OF);
    for (int i = 0; i < OF.rows(); i++) {
        cout << "F_ : " << F_.row(i) << " OF : " << OF.row(i) << endl;
    }*/
    // check whether mesh is manifold
    //if(is_edge_manifold(OF)) {
    if(qem::is_manifold(OV, OF)){
        cout << "\n" << "*******************************" << endl;
        cout << "Input model is Manifold mesh" << endl;
        cout << "Number of Vertex : " << OV.rows() << endl;
        cout << "Number of Faces : " << OF.rows() << endl;
        cout << "*******************************" << "\n" << endl;
    }
    else {
        cout << "Input model is Non-Manifold mesh" << endl;
        cout << "Please use Manifold mesh" << endl;
        return -1;
    }
    // Prepare array-based edge data structures and priority queue
    VectorXi EMAP, EQ;
    MatrixXi E,EF,EI;
    igl::min_heap< std::tuple<double,int,int> > Q;
    // Set stopping condition based on removed vertices
    int num_input_vertices = OV.rows();
    int stopping_condition = ceil(num_input_vertices * (1.0 - ratio));
    //initialize viewer
    igl::opengl::glfw::Viewer viewer;
    // If an edge were collapsed, we'd collapse it to these points:
    MatrixXd C;
    int num_collapsed;

    // Surface normal per vertex to compute Q
    Eigen::MatrixXd N_homo(OF.rows(), 4);
    qem::get_normal_homo_per_face(OV, OF, N_homo);

    // Add initial Q value of each vertex to vector
    qem::QValues qValues;
    qem::init_qValues(OV, OF, N_homo, qValues.values);

    edge_flaps(OF, E, EMAP, EF, EI);

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
        qem::quadratic(e, V, F, E, EMAP, EF, EI, qValues.values, cost, p);
    };
    // call wrap function to use qValues in callback function
    //customCBF::setup_cost_and_placement_with_qValues(qValues);
    qem::setup_post_collapse_with_qValues(qValues, Q);

    const auto &process = [&](igl::opengl::glfw::Viewer & viewer)->bool
    {
        clock_t start, end;
        start = clock();
        while(!Q.empty())
        {
            bool something_collapsed = false;
            bool flag = false;
            // collapse edge
            const int max_iter = std::ceil(0.01*Q.size());
            for(int j = 0;j<max_iter;j++)
            {
                // if collapsing doesn't occur, break
                if(!collapse_edge(quadratic_with_qValues,
                                  qem::pre_collapse,
                                  qem::post_collapse,
                                  V, F, E, EMAP, EF, EI, Q, EQ, C))
                {
                    break;
                }
                something_collapsed = true;
                num_collapsed++;
                //cout << num_collapsed << " vertices are collapsed\n" << endl;
                // if stopping condition met, break
                if (num_collapsed>=stopping_condition) {
                    flag = true;
                    // remove duplicated vertices and faces
                    end = clock();
                    qem::remove_duplicated_faces(V, F);
                    cout << "\n" << "*******************************" << endl;
                    if (qem::is_manifold(V, F)) cout << "Resulting mesh is Manifold" << endl;
                    else { cout << "Resulting mesh is Non-Manifold" << endl; }
                    cout << "*******************************" << endl;
                    break;
                }
            }

            if(something_collapsed)
            {
                if(flag) {
                    cout << "total time : " << (double) (end - start) / CLOCKS_PER_SEC << " second" << endl;
                    if(writeOBJ(OUTPUT_PATH + output_filename + ".obj", V, F)){
                        cout << "Successfully wrote to " << output_filename << ".obj" << endl;
                        return 0;
                    }
                    else{
                        cout << "Failed to wrote to " << output_filename << ".obj" << endl;
                        return -1;
                    }
                }
            }
        }
        return false;
    };

    /*
    const auto &pre_draw = [&](igl::opengl::glfw::Viewer & viewer)->bool
    {
        // If animating then collapse 10% of edges
        if(viewer.core().is_animating && !Q.empty())
        {
            bool something_collapsed = false;
            bool flag = false;
            // collapse edge
            const int max_iter = std::ceil(0.01*Q.size());
            for(int j = 0;j<max_iter;j++)
            {
                // if collapsing doesn't occur, break
                if(!collapse_edge(quadratic_with_qValues,
                                  qem::pre_collapse,
                                  qem::post_collapse,
                                  V, F, E, EMAP, EF, EI, Q, EQ, C))
                {
                    break;
                }
                something_collapsed = true;
                num_collapsed++;
                // if stopping condition met, break
                if (num_collapsed>=stopping_condition) {
                    viewer.core().is_animating = false;
                    flag = true;
                    // remove duplicated vertices and faces
                    qem::remove_duplicated_faces(V, F);
                    cout << "\n" << "*******************************" << endl;
                    if(is_edge_manifold(F)) cout << "Resulting mesh is Manifold" << endl;
                    else cout << "Resulting mesh is Non-Manifold" << endl;
                    cout << "*******************************" << endl;
                    break;
                }
            }

            if(something_collapsed)
            {
                cout << num_collapsed << " vertices are removed" << endl;
                viewer.data().clear();
                viewer.data().set_mesh(V,F);
                viewer.data().set_face_based(true);
                if(flag) {
                    if(writeOBJ(OUTPUT_PATH + output_filename + ".obj", V, F)){
                        cout << "Successfully wrote to " << output_filename << ".obj" << endl;
                        return 0;
                    }
                    else{
                        cout << "Failed to wrote to " << output_filename << ".obj" << endl;
                        return -1;
                    }
                }
            }
        }
        return false;
    };

    const auto &key_down =
            [&](igl::opengl::glfw::Viewer &viewer,unsigned char key,int mod)->bool
            {
                if(key ==' ') viewer.core().is_animating ^= 1;
                else return false;
                return true;
            };
*/

    //reset();
    // reset function to assign all initial value in min heap, especially cost_table (qValues.values)
    qem::reset(V, OV, F, OF, E, EMAP, EF, EI, EQ, C, Q,
               qValues.values, viewer, num_collapsed);
    //viewer.core().background_color.setConstant(1);
/*    qem::process(quadratic_with_qValues,
                 qem::pre_collapse,
                 qem::post_collapse,
                 V,
                 F,
                 E,
                 EMAP,
                 EF,
                 EI,
                 EQ,
                 C,
                 Q,
                 stopping_condition,
                 num_collapsed
    );*/
    //viewer.core().is_animating = true;
    //viewer.callback_key_down = key_down;
    //viewer.callback_pre_draw = pre_draw;
    process(viewer);

    // Erase this for docker image
    //viewer.data().clear();
    //viewer.data().set_mesh(V,F);
    //viewer.data().set_face_based(true);

    //return viewer.launch();
    return 0;
}