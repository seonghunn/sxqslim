//
// Created by seonghun on 7/24/23.
//

#include "preprocess.h"

namespace qslim{
    void set_input_orient_outward(MatrixXd &V, MatrixXi &F, MatrixXi &FF) {
/*      VectorXi C, I;
        MatrixXi FF;
        igl::facet_components(F, C);

        cout << C << endl;

        igl::orient_outward(V, F, C, FF, I);

        return FF;*/
        if (qslim::check_mesh_orientation(V, F)) {
            FF = F;
            return;
        }
        FF.resize(F.rows(), F.cols());

        for (int i = 0; i < F.rows(); ++i) {
            Eigen::Vector3d v1 = V.row(F(i, 0));
            Eigen::Vector3d v2 = V.row(F(i, 1));
            Eigen::Vector3d v3 = V.row(F(i, 2));
            Eigen::Vector3d centroid = (v1 + v2 + v3) / 3;
            Eigen::Vector3d normal = (v2 - v1).cross(v3 - v1);

            // dot product using normal and centroid vector, if it is positive, the surface orients outward.
            if (centroid.dot(normal) < 0) {
                FF.row(i) << F(i, 0), F(i, 2), F(i, 1);
            }
            else{
                FF.row(i) << F.row(i);
            }
        }
    }

    void get_normal_homo_per_face(MatrixXd &V, MatrixXi &F, MatrixXd &N_homo){
        Eigen::MatrixXd N_faces;
        igl::per_face_normals(V, F, N_faces);

        // Transform to Homogeneous Coordinate
        for (int i = 0; i < N_faces.rows(); i++) {
            Eigen::RowVector3d n = N_faces.row(i);
            double d = -n.dot(V.row(F(i, 0)));
            N_homo.row(i) << n, d;
        }
    }

    void init_qValues(MatrixXd &V, MatrixXi &F, MatrixXd &N_homo, std::vector<Matrix4d> &table){
        table.resize(V.rows(), Eigen::Matrix4d::Zero());
        for (int i = 0; i < F.rows(); i++) {
            Eigen::Vector4d p(N_homo(i, 0), N_homo(i, 1), N_homo(i, 2), N_homo(i, 3));
            Eigen::Matrix4d q = p * p.transpose();
            // Add q for each 3 vertex in face, addition for summing q for all adjacent planes
            for (int j = 0; j < 3; j++) {
                table[F(i, j)] += q;
            }
        }
    }
}