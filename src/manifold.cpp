//
// Created by seonghun on 7/23/23.
//

#include "manifold.h"

namespace qem{
    //TODO: self orientation doesn't work at large mesh
    bool check_mesh_orientation(const MatrixXd &V, const MatrixXi &F){
        for (int i = 0; i < F.rows(); ++i) {
            Eigen::Vector3d v1 = V.row(F(i, 0));
            Eigen::Vector3d v2 = V.row(F(i, 1));
            Eigen::Vector3d v3 = V.row(F(i, 2));
            Eigen::Vector3d centroid = (v1 + v2 + v3) / 3;
            Eigen::Vector3d normal = (v2 - v1).cross(v3 - v1);

            // dot product using normal and centroid vector, if it is positive, the surface orients outward.
            if (centroid.dot(normal) < 0) {
                return false;
            }
        }
        return true;
    }

    //TODO: Implement self intersection
    bool check_self_intersection(const MatrixXd &V, const MatrixXi &F){
        //igl::copyleft::cgal::RemeshSelfIntersectionsParam params;
        //params.detect_only = true;

        Eigen::MatrixXd V_temp;
        Eigen::MatrixXi F_temp,IF,J,IM;

        //igl::copyleft::cgal::remesh_self_intersections(V, F, params, V_temp, F_temp, IF, J, IM);

        cout << "IF " << IF << endl;

        return true;
    }

    bool is_manifold(const MatrixXd &V, const MatrixXi &F){
        // check edge_manifold
        if(!igl::is_edge_manifold(F)) {
            cout<< "edge manifold test fail"<<endl;
            return false;
        }

        cout << "edge manifold test success" << endl;

/*
        // check orientation
        if(!qem::check_mesh_orientation(V, F)){
            cout << "orientation test fail" << endl;
            return false;
        }

        cout << "orientation test success" << endl;
*/


        //TODO: check self-intersection
        //check self-intersection
        check_self_intersection(V, F);

        return true;
    }
}