//
// Created by seonghun on 8/22/23.
//

#include "MeshSimplify.h"
#include <algorithm>

namespace qslim{
    MeshSimplify::MeshSimplify(MatrixXd &OV, MatrixXi &OF) {
        cout << "# of Input Vertex : " << OV.rows() << "\n";
        cout << "# of Input Face : " << OF.rows() << "\n";
        start = clock();
        this->init_member_variable(OV, OF);
        this->tree = aabb::Tree(3, 0, 16, false);
        qslim::initialize_tree_from_mesh(OV, OF, this->tree);
        this->init_normal_homo_per_face(OV, OF, this->N_homo);
        this->init_qValues(OV, OF, this->N_homo);
        this->init_queue();
        this->init_callback();
        end = clock();
        cout << "Initialize tree done : " << (double) (end - start) / CLOCKS_PER_SEC << " sec" << endl;
        start = clock();
        this->input_manifold_test(OV, OF);
        end = clock();
        cout << "Input manifold test done : " << (double) (end - start) / CLOCKS_PER_SEC << " sec" << endl;
    }

    bool MeshSimplify::input_manifold_test(MatrixXd &OV, MatrixXi &OF){
        if (is_manifold(OV, OF, this->tree, true)) {
            //cout << "Input is manifold mesh\n";
            return true;
        }
        else {
            cout << "Input is Non-Manifold mesh" << endl;
            cout << "Please use Manifold mesh" << endl;
            return false;
        }
    }

    void MeshSimplify::init_member_variable(MatrixXd &OV, MatrixXi &OF){
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
        this->num_collapsed = 0;
        this->num_failed = 0;
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

    void MeshSimplify::init_queue(){
        int num = this->E.rows();
        VectorXd costs(num);
        for (int e=0; e < num; e++) {
            double cost = e;
            RowVectorXd p(1, 3);
            qslim::quadratic(e, this->V, this->F, this->E, this->EMAP,
                             this->EF, this->EI, this->qValues, cost, p);
            this->C.row(e) = p;
            costs(e) = cost;
            this->queue.emplace(costs(e), e, 0);
        }

        this->num_collapsed = 0;
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
            // e : collapsed edge index

            // return false if manifold test fails
            // if this function returns false, then the candidate edge is not gonna be collapse
            // after return false, assign infinity cost for that edge

            // copy variable to pre-collapse
            MatrixXd V_ = V;
            MatrixXi F_ = F;
            MatrixXi E_ = E;
            VectorXi EMAP_ = EMAP;
            MatrixXi EF_ = EF;
            MatrixXi EI_ = EI;
            RowVectorXd p = this->C.row(e); // placement when collapsing edge e

            // vertices of collapsed edge
            int RV_idx1 = E(e, 0);
            int RV_idx2 = E(e, 1);

            // get restore values before edge collapse
            vector<int> oneRingNeigh;
            get_one_ring_neigh(e, this->F, this->EMAP, this->EF, this->EI, oneRingNeigh);
            takeNodeSnapShot(oneRingNeigh, this->tree, this->restoreMap);

            // pre collapse
            igl::collapse_edge(e, p, V_, F_, E_, EMAP_, EF_, EI_);
            // update tree based on edge collapse
            update_tree_after_decimation(V_, F_, this->tree, RV_idx1, RV_idx2, oneRingNeigh);
            // self intersection check
            if (!qslim::is_manifold(V_, F_, this->tree, oneRingNeigh, RV_idx1, RV_idx2, false)){
                //std::cout<<"pre collapse false"<<std::endl;
                return false;
            }
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

            // if collapse is failed, restore the tree
            if(!collapsed){
                vector<int> oneRingNeigh;
                get_one_ring_neigh(e, this->F, this->EMAP, this->EF, this->EI, oneRingNeigh);
                restoreTree(oneRingNeigh, this->restoreMap, this->tree, f1, f2);
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

        // Optimal
/*        Eigen::Vector4d target;
        if(A.determinant() > 1e-5){
            target = A.inverse() * Eigen::Vector4d(0, 0, 0, 1);
            p = target.head<3>() / target.w();
        }
        else{
            p = (V.row(v1) + V.row(v2)) / 2.0;
            target << p.transpose(), 1;
        }
        Eigen::MatrixXd result = target.transpose() * Q * target;
        cost = std::max(0.0, result(0, 0));*/


        // midpoint
        p = (V.row(v1) + V.row(v2)) / 2.0;
        Vector4d p_homogeneous;
        p_homogeneous << p.transpose(), 1;
        Eigen::MatrixXd result = p_homogeneous.transpose() * Q * p_homogeneous; // 이 연산의 결과는 1x1 행렬입니다.
        cost = std::max(0.0, result(0, 0));
    }

    void MeshSimplify::process(double ratio){
        this->ratio = ratio;
        this->stopping_condition = ceil(this->num_input_vertices * (1.0 - this->ratio));

        auto cost_and_placement_callback = this->get_cost_and_position_callback();
        auto pre_collapse_callback = this->get_pre_collapse_callback();
        auto post_collapse_callback = this->get_post_collapse_callback();

        this->start = clock();
        int iteration = 0;
        while (!this->queue.empty()) {
            // TODO: is num failed ok?
            // break loop
            if (this->num_collapsed >= this->stopping_condition || num_failed > 10 * this->OV.rows()) {
                this->end = clock();
                break;
            }
            if(iteration % 5000 == 0 && iteration != 0)
                cout << "Iteration : " << iteration << endl;

            bool collapsed = igl::collapse_edge(cost_and_placement_callback,
                                    pre_collapse_callback,
                                    post_collapse_callback,
                                    this->V, this->F, this->E, this->EMAP,
                                    this->EF, this->EI, this->queue, this->EQ, this->C);
            iteration ++;
            if(collapsed)
                this->num_collapsed++;
            else
                this->num_failed++;
        }
/*        if (qslim::is_manifold(this->V, this->F, this->tree, true))
            cout << "Resulting mesh is Manifold" << endl;
        else
            cout << "Resulting mesh is Non-Manifold" << endl;*/
        // Remove duplicated faces
        qslim::remove_duplicated_faces(this->V, this->F);
        cout << "Output V : " << this->V.rows() << endl;
        cout << "Output F : " << this->F.rows() << endl;
        cout << "total runtime : " << (double) (this->end - this->start) / CLOCKS_PER_SEC << " second" << endl;
    }
}