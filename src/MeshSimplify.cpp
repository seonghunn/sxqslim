//
// Created by seonghun on 8/22/23.
//

#include "MeshSimplify.h"

#define INPUT_PATH "../model/input/"
#define OUTPUT_PATH "../model/output/"

namespace qslim{
    MeshSimplify::MeshSimplify(MatrixXd &OV, MatrixXi &OF, double ratio){
        this->init_member_variable(OV, OF, ratio);
        this->tree = aabb::Tree(3, 0.0001, 16, false);
        qslim::initialize_tree_from_mesh(OV, OF, this->tree);
        this->init_normal_homo_per_face(OV, OF, this->N_homo);
        this->init_qValues(OV, OF, this->N_homo);
        this->init_queue(OV, OF);
        this->init_callback();
    }

    void MeshSimplify::init_member_variable(MatrixXd &OV, MatrixXi &OF, double ratio){
        this->OV = OV;
        this->OF = OF;
        this->V = OV;
        this->F = OF;
        igl::edge_flaps(this->F, this->E, this->EMAP, this->EF, this->EI);
        this->EQ = Eigen::VectorXi::Zero(this->E.rows());
        this->C.resize(this->E.rows(), this->V.cols());
        this->queue={};
        this->num_input_vertices = OV.rows();
        this->num_input_faces = OF.rows();
        this->ratio = ratio;
        this->num_collapsed = 0;
        this->num_failed = 0;
        this->stopping_condition = ceil(this->num_input_vertices * (1.0 - this->ratio));

        //init hash map for deleted faces
        for (int i = 0; i < this->num_input_faces; i++) {
            this->decimated_faces.insert(make_pair(i, false));
        }

        //init map for affect_triangles
        this->init_affect_triangles(OV, OF);
    }

    void MeshSimplify::init_affect_triangles(MatrixXd &OV, MatrixXi &OF){
        for (int i = 0; i < OV.rows(); i++) {
            vector<int> triangle_list;
            for (int j = 0; j < OF.rows(); j++) {
                for (int k = 0; k < 3; k++) {
                    if (i == OF(j, k)) {
                        triangle_list.push_back(j);
                    }
                }
            }
            this->affected_triangle_indices.insert(make_pair(i, triangle_list));
        }
    }

    void MeshSimplify::init_normal_homo_per_face(MatrixXd &OV, MatrixXi &OF, MatrixXd &N_homo){
        Eigen::MatrixXd N_faces;
        N_homo = MatrixXd(OF.rows(), 4);
        igl::per_face_normals(OV, OF, N_faces);
        // Transform to Homogeneous Coordinate
        for (int i = 0; i < N_faces.rows(); i++) {
            Eigen::RowVector3d n = N_faces.row(i);
            double d = -n.dot(OV.row(OF(i, 0)));
            N_homo.row(i) << n, d;
        }
    }

    void MeshSimplify::init_qValues(MatrixXd &V, MatrixXi &F, MatrixXd &N_homo){
        this->qValues.resize(V.rows(), Eigen::Matrix4d::Zero());
        for (int i = 0; i < F.rows(); i++) {
            Eigen::Vector4d p(N_homo(i, 0), N_homo(i, 1), N_homo(i, 2), N_homo(i, 3));
            Eigen::Matrix4d q = p * p.transpose();
            // Add q for each 3 vertex in face, addition for summing q for all adjacent planes
            for (int j = 0; j < 3; j++) {
                this->qValues[F(i, j)] += q;
            }
        }
    }

    void MeshSimplify::init_queue(MatrixXd &OV,MatrixXi &OF) {
        {
            Eigen::VectorXd costs(this->E.rows());
            //parallel code
            igl::parallel_for(this->E.rows(), [&](const int e) {
                double cost = e;
                RowVectorXd p(1, 3);
                //reset each cost
                qslim::quadratic(e, this->V, this->F, this->E, this->EMAP,
                                 this->EF, this->EI, this->qValues, cost, p);
                this->C.row(e) = p;
                costs(e) = cost;
            }, 10000);
            for (int e = 0; e < this->E.rows(); e++) {
                this->queue.emplace(costs(e), e, 0);
            }
            this->num_collapsed = 0;
            this->viewer.data().clear();
            this->viewer.data().set_mesh(V, F);
            this->viewer.data().set_face_based(true);
        }
    }

    void MeshSimplify::init_callback() {
        this->cost_and_position_callback = [this](
                const int e,
                const Eigen::MatrixXd & /*V*/,
                const Eigen::MatrixXi & /*F*/,
                const Eigen::MatrixXi & /*E*/,
                const Eigen::VectorXi & /*EMAP*/,
                const Eigen::MatrixXi & /*EF*/,
                const Eigen::MatrixXi & /*EI*/,
                double & cost,
                Eigen::RowVectorXd & p)
        {
            this->cost_and_position(e, this->V, this->F, this->E, this->EMAP, this->EF, this->EI, cost, p);
        };

        this->pre_collapse = [this](
                const Eigen::MatrixXd &,               // V: Vertices of the mesh
                const Eigen::MatrixXi &,                // F: Faces of the mesh
                const Eigen::MatrixXi &,               // E: Edges of the mesh
                const Eigen::VectorXi &,            // EMAP: Edge Map (likely mapping edge indices)
                const Eigen::MatrixXi &,              // EF: Edge to Face adjacency
                const Eigen::MatrixXi &,              // EI: Edge information (possibly edge to vertex)
                const igl::min_heap< std::tuple<double,int,int> > &,  // Q: Priority queue of edges to collapse (with costs)
                const Eigen::VectorXi &,              // EQ: Unknown, possibly edge quality or equivalence
                const Eigen::MatrixXd &,               // C: Unknown, possibly color or curvature data
                const int e                            // e: Edge index to collapse
                ) -> bool
        {
            // TODO: Your custom logic here. For example, you might want to prevent
            // collapsing edges that are on the boundary of the mesh:
            // if (is_boundary_edge(e, V, F, E, EMAP, EF, EI)) {
            //     return false;
            // }
            //Add this logic : total time complexity -> O(N^2 log(N))
            // collapsing edge candidate
                MatrixXd V_ = V;
                MatrixXi F_ = F;
                MatrixXi E_ = E;
                VectorXi EMAP_ = EMAP;
                MatrixXi EF_ = EF;
                MatrixXi EI_ = EI;
                RowVectorXd p = V.row(E(e, 0));
                clock_t start_remove, end_remove, start_test, end_test, start_collapse, end_collapse;
                start_collapse = clock();
                igl::collapse_edge(e, p, V_, F_, E_, EMAP_, EF_, EI_);
                end_collapse = clock();

                start_remove = clock();
                //qslim::remove_duplicated_faces(V_, F_);
                end_remove = clock();
                start_test = clock();

                if(!qslim::is_manifold(V, F, this->tree, this->decimated_faces))
                    return false;
                end_test = clock();
                cout << "pre - collapsing edge : " << (double) (end_collapse - start_collapse) / CLOCKS_PER_SEC << " sec" << endl;
                //cout << "remove duplicated faces : " << (double) (end_remove - start_remove) / CLOCKS_PER_SEC << " sec" << endl;
                cout << "total test : " << (double) (end_test - start_test) / CLOCKS_PER_SEC << " sec" << endl;
                //cout << "Before collapsing number of vertices : " << V_.rows() << endl;
            // Get index of vertices which supposed to be replaced
            //TODO: 여기서 false 일 때 cost 를 infinite 로
            this->RV.v1 = E(e,0);
            this->RV.v2 = E(e,1);

            return true;  // Allow the edge to be collapsed.
        };

        this->post_collapse = [this](
                const Eigen::MatrixXd &V,
                const Eigen::MatrixXi &F,
                const Eigen::MatrixXi &E,
                const Eigen::VectorXi &EMAP,
                const Eigen::MatrixXi &EF,
                const Eigen::MatrixXi &EI,
                const igl::min_heap<std::tuple<double, int, int> > &,
                const Eigen::VectorXi &EQ,
                const Eigen::MatrixXd &C,
                const int e,
                const int e1,
                const int e2,
                const int f1,
                const int f2,
                const bool collapsed
                ) {
/*                    cout << "V" << endl;
                    qslim::printMatrix(V);
                    cout << "F" << endl;
                    qslim::printMatrix(F);
                    cout << "E" << endl;
                    qslim::printMatrix(E);
                    cout << "EF" << endl;
                    qslim::printMatrix(EF);
                    cout << "EI" << endl;
                    qslim::printMatrix(EI);
                    cout << "C" << endl;
                    qslim::printMatrix(C);
                    cout << "EMAP : " << EMAP << endl;
                    cout << "e : " << e << endl;
                    cout << "e1 : " << e1 << endl;
                    cout << "e2 : " << e2 << endl;
                    cout << "f1 : " << f1 << endl;
                    cout << "f2 : " << f2 << endl;*/
            if (collapsed) {
                int RV_idx1 = this->RV.v1;
                int RV_idx2 = this->RV.v2;
                // update qValues
                Eigen::Matrix4d Q1 = this->qValues[RV_idx1];
                Eigen::Matrix4d Q2 = this->qValues[RV_idx2];
                this->qValues[RV_idx1] = Q1 + Q2;
                this->qValues[RV_idx2] = Q1 + Q2;
                // update AABB tree
                qslim::update_tree_after_decimation(this->V, this->F, this->tree,
                                                    RV_idx1, RV_idx2, f1, f2, this->decimated_faces,
                                                    this->affected_triangle_indices);
                // update Decimated faces table
                this->decimated_faces[f1] = true;
                this->decimated_faces[f2] = true;
            }
        };
    }

    MatrixXd MeshSimplify::get_vertices(){
        return this->V;
    }

    MatrixXi MeshSimplify::get_faces() {
        return this->F;
    }

    igl::decimate_cost_and_placement_callback MeshSimplify::get_cost_and_position_callback() {
        return this->cost_and_position_callback;
    }

    igl::decimate_pre_collapse_callback MeshSimplify::get_pre_collapse_callback() {
        return this->pre_collapse;
    }

    igl::decimate_post_collapse_callback MeshSimplify::get_post_collapse_callback() {
        return this->post_collapse;
    }

    void MeshSimplify::cost_and_position(
            const int e,
            const Eigen::MatrixXd & V,
            const Eigen::MatrixXi & /*F*/,
            const Eigen::MatrixXi & E,
            const Eigen::VectorXi & /*EMAP*/,
            const Eigen::MatrixXi & /*EF*/,
            const Eigen::MatrixXi & /*EI*/,
            double & cost,
            Eigen::RowVectorXd & p) {
        // E(e,0) returns the index of first vertex of edge e
        int v1 = E(e, 0);
        int v2 = E(e, 1);
        Eigen::Matrix4d Q = this->qValues[v1] + this->qValues[v2];
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
    }

    bool MeshSimplify::process(){
        auto cost_and_placement_callback = this->get_cost_and_position_callback();
        auto pre_collapse_callback = this->get_pre_collapse_callback();
        auto post_collapse_callback = this->get_post_collapse_callback();

        clock_t start, end;
        start = clock();
        while (!this->queue.empty()) {
            cout << "\niteration : " << this->num_collapsed << endl;
            // collapse edge
/*            cout << "collapsed : " << this->num_collapsed << endl;
            cout << "decimated index of vertex" << endl;
            cout << this->RV.v1 << " " << this->RV.v2 << endl;
            cout << "********V*********" << endl;
            cout << this->V << endl;
            cout << "********F*********" << endl;
            cout << this->F << endl;
            cout << "********E*********" << endl;
            cout << this->E << endl;
            cout << endl;*/
            // if stopping condition met, break
            bool collapsed = igl::collapse_edge(cost_and_placement_callback,
                                    pre_collapse_callback,
                                    post_collapse_callback,
                                    this->V, this->F, this->E, this->EMAP,
                                    this->EF, this->EI, this->queue, this->EQ, this->C);

            //cout << "true / false " << collapsed << endl;
            this->num_collapsed++;
            //cout << num_collapsed << " vertices are collapsed\n" << endl;
            if (this->num_collapsed >= this->stopping_condition) {
                // remove duplicated vertices and faces
                end = clock();
                break;
            }
        }
        qslim::remove_duplicated_faces(this->V, this->F);
        cout << "\n" << "*******************************" << endl;
        cout << "Output V : " << this->V.rows() << endl;
        cout << "Output F : " << this->F.rows() << endl;
        if (qslim::is_manifold(this->V, this->F, this->tree, this->decimated_faces))
            cout << "Resulting mesh is Manifold" << endl;
        else
            cout << "Resulting mesh is Non-Manifold" << endl;
        cout << "*******************************" << endl;
        cout << "total time : " << (double) (end - start) / CLOCKS_PER_SEC << " second" << endl;
        // write file
        if (igl::writeOBJ(OUTPUT_PATH + this->output_filename + ".obj", this->V, this->F)) {
            cout << "Successfully wrote to " << this->output_filename << ".obj" << endl;
            return 0;
        }
        else {
            cout << "Failed to wrote to " << this->output_filename << ".obj" << endl;
            return -1;
        }
    }
}