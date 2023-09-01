//
// Created by seonghun on 8/22/23.
//

#include "MeshSimplify.h"

#define INPUT_PATH "../model/input/"
#define OUTPUT_PATH "../model/output/"

namespace qslim{
    MeshSimplify::MeshSimplify(MatrixXd &OV, MatrixXi &OF, double ratio, string output_filename) {
        this->init_member_variable(OV, OF, ratio, output_filename);
        this->tree = aabb::Tree(3, 0, 16, false);
        qslim::initialize_tree_from_mesh(OV, OF, this->tree);
        this->init_normal_homo_per_face(OV, OF, this->N_homo);
        this->init_qValues(OV, OF, this->N_homo);
        this->init_queue(OV, OF);
        this->init_callback();
        this->input_manifold_test(OV, OF);
    }

    bool MeshSimplify::input_manifold_test(MatrixXd &OV, MatrixXi &OF){
        cout << "\n" << "*******************************\n";
        cout << "Number of Input Vertex : " << OV.rows() << "\n";
        cout << "Number of Input Faces : " << OF.rows() << "\n";
        if (is_manifold(OV, OF, this->tree, this->decimated_faces, true)) {
            cout << "Input model is Manifold mesh\n";
            cout << "*******************************" << "\n\n";
            return true;
        }
        else {
            cout << "Input model is Non-Manifold mesh" << endl;
            cout << "Please use Manifold mesh" << endl;
            cout << "*******************************" << "\n\n";
            return false;
        }
    }

    void MeshSimplify::init_member_variable(MatrixXd &OV, MatrixXi &OF, double ratio, string output_filename){
        this->output_filename = output_filename;
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
                const Eigen::VectorXi &,              // EQ: Timestamps for each edge
                const Eigen::MatrixXd &,               // C: Placement of an edge
                const int e                            // e: Stores the optimal position data for each edge in the event it's collapsed.
                ) -> bool
        {
            // return false if manifold test fails
            // if this function returns false, then the candidate edge is not gonna be collapse
            // after return false, assign infinity cost for that edge
            MatrixXd V_ = V;
            MatrixXi F_ = F;
            MatrixXi E_ = E;
            VectorXi EMAP_ = EMAP;
            MatrixXi EF_ = EF;
            MatrixXi EI_ = EI;
            RowVectorXd p = this->C.row(e); // placement when collapsing edge e

            int RV_idx1 = E(e, 0);
            int RV_idx2 = E(e, 1);
            int removedFaceIdx1;
            int removedFaceIdx2;

            vector<int> tmpFaceList;
            for (int i: this->affected_triangle_indices[RV_idx1]) {
                for (int j: this->affected_triangle_indices[RV_idx2]) {
                    if(i==j) tmpFaceList.push_back(i);
                }
            }

            // if decimated face is not 2, this leads to invalid mesh
            if(tmpFaceList.size()!=2) return false;

            // index of decimated faces
            removedFaceIdx1 = tmpFaceList[0];
            removedFaceIdx2 = tmpFaceList[1];

            // buffer for affected triangle indices (to restore)
            vector<int> affected_triangle_indices_tmp1 = this->affected_triangle_indices[RV_idx1];
            vector<int> affected_triangle_indices_tmp2 = this->affected_triangle_indices[RV_idx2];

            // update Decimated faces table
            this->decimated_faces[removedFaceIdx1] = true;
            this->decimated_faces[removedFaceIdx2] = true;

            vector<int> combinedAffectedTriangleIndices;
            updateAffectedTriangle(this->affected_triangle_indices, this->decimated_faces, RV_idx1, RV_idx2,
                                   combinedAffectedTriangleIndices);

            unordered_map<int, NodeSnapshot> restoreMap;
            takeNodeSnapShot(combinedAffectedTriangleIndices, this->tree, removedFaceIdx1, removedFaceIdx2, restoreMap);

            clock_t start_test, end_test, start_collapse, end_collapse;
            start_collapse = clock();
            igl::collapse_edge(e, p, V_, F_, E_, EMAP_, EF_, EI_);
            end_collapse = clock();
            start_test = clock();

            // update tree after decimation
            update_tree_after_decimation(V_, F_, this->tree, RV_idx1, RV_idx2, removedFaceIdx1, removedFaceIdx2, this->decimated_faces,
                                         combinedAffectedTriangleIndices);
            // if test failed, restore tree
            if (!qslim::is_manifold(V_, F_, this->tree, this->decimated_faces,
                                    this->affected_triangle_indices, RV_idx1, RV_idx2, false)){
                restoreTree(combinedAffectedTriangleIndices, restoreMap, this->tree, removedFaceIdx1, removedFaceIdx2);

                //restore affected triangle indices
                this->affected_triangle_indices[RV_idx1] = affected_triangle_indices_tmp1;
                this->affected_triangle_indices[RV_idx2] = affected_triangle_indices_tmp2;

                // combined : set of affected triangles except decimated triangles (faces)
                // need to update for decimated faces
                // restore Decimated faces table
                this->decimated_faces[removedFaceIdx1] = false;
                this->decimated_faces[removedFaceIdx2] = false;

                return false;
            }

                end_test = clock();
                //cout << "pre - collapsing edge : " << (double) (end_collapse - start_collapse) / CLOCKS_PER_SEC << " sec" << endl;
                //cout << "remove duplicated faces : " << (double) (end_remove - start_remove) / CLOCKS_PER_SEC << " sec" << endl;
                cout << "total test : " << (double) (end_test - start_test) / CLOCKS_PER_SEC << " sec\n";
                //cout << "Before collapsing number of vertices : " << V_.rows() << endl;
            // Get index of vertices which supposed to be replaced

            this->RV.v1 = RV_idx1;
            this->RV.v2 = RV_idx2;

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

            // if edge was collapsed, update qValues table
            if (collapsed) {
                int RV_idx1 = this->RV.v1;
                int RV_idx2 = this->RV.v2;
                // update qValues
                Eigen::Matrix4d Q1 = this->qValues[RV_idx1];
                Eigen::Matrix4d Q2 = this->qValues[RV_idx2];
                this->qValues[RV_idx1] = Q1 + Q2;
                this->qValues[RV_idx2] = Q1 + Q2;
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

        Eigen::Vector4d target;
        if(A.determinant() < 1e-5){
            target = A.inverse() * Eigen::Vector4d(0, 0, 0, 1);
            p = target.head<3>() / target.w();
        }
        else{
            p = (V.row(v1) + V.row(v2)) / 2.0;
            target << p.transpose(), 1;
        }
        cost = target.transpose() * Q * target;
        // transform homogeneous coordinates to normal coordinates
/*
        p = target.head<3>() / target.w();
        cost = target.transpose() * Q * target;
*/

        // midpoint
/*        p = (V.row(v1) + V.row(v2)) / 2.0;
        Vector4d p_homogeneous;
        p_homogeneous << p.transpose(), 1;
        cost = p_homogeneous.transpose() * Q * p_homogeneous;*/
    }

    bool MeshSimplify::process(){
        auto cost_and_placement_callback = this->get_cost_and_position_callback();
        auto pre_collapse_callback = this->get_pre_collapse_callback();
        auto post_collapse_callback = this->get_post_collapse_callback();

        clock_t start, end;
        start = clock();
        int iteration = 0;
        while (!this->queue.empty()) {
            // collapse edge, print data
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
            iteration ++;
            if(collapsed)
                this->num_collapsed++;
            else
                this->num_failed++;
            //cout << num_collapsed << " vertices are collapsed\n" << endl;
            cout << "\niteration : " << iteration << " num - collapsed : " << this->num_collapsed << "\n";
            // TODO: 여기 num failed 말고 큐가 비면 끝나게 해야함
            if (this->num_collapsed >= this->stopping_condition || num_failed > 10 * this->OV.rows()) {
                // remove duplicated vertices and faces
                end = clock();
                break;
            }
        }
        //qslim::remove_duplicated_faces(this->V, this->F);
        cout << "\n" << "*******************************" << endl;
        if (qslim::is_manifold(this->V, this->F, this->tree, this->decimated_faces, true))
            cout << "Resulting mesh is Manifold" << endl;
        else
            cout << "Resulting mesh is Non-Manifold" << endl;
        // Remove duplicated faces
        qslim::remove_duplicated_faces(this->V, this->F);
        cout << "Output V : " << this->V.rows() << endl;
        cout << "Output F : " << this->F.rows() << endl;
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