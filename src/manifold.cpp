//
// Created by seonghun on 7/23/23.
//

#include "manifold.h"
#include <time.h>

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

        Eigen::MatrixXi intersect, edges;

        //if it returns false, it means the mesh does not have self-intersections
       return igl::fast_find_self_intersections(V, F, intersect);
       //return true;
    }

    bool is_manifold(const MatrixXd &V, const MatrixXi &F){
        // check edge_manifold
        clock_t start_edge, end_edge, start_intersect, end_intersect;
        start_edge = clock();
        if(!igl::is_edge_manifold(F)) {
            //cout<< "edge manifold test fail"<<endl;
            return false;
        }
        end_edge = clock();
        cout << "edge manifold test : " << (double) (end_edge - start_edge) / CLOCKS_PER_SEC << " sec" << endl;
        //cout << "edge manifold test success" << endl;

        //check self-intersection
        start_intersect = clock();
        if(check_self_intersection(V, F)){
            // self intersection exist
            //cout << "self-intersection test fail" << endl;
            return false;
        }
        end_intersect = clock();
        cout << "self intersect test : " << (double) (end_intersect - start_intersect) / CLOCKS_PER_SEC << " sec" << endl;
        //cout << "self-intersection test success" << endl;

/*        // check orientation
        if(!qem::check_mesh_orientation(V, F)){
            cout << "orientation test fail" << endl;
            return false;
        }

        cout << "orientation test success" << endl;*/


        return true;
    }
}