//
// Created by seonghun on 7/23/23.
//

#include "manifold.h"

/*namespace PMP = CGAL::Polygon_mesh_processing;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;*/

namespace qslim{
    bool check_edge_manifold(const MatrixXd &V, const MatrixXi &F){
        MatrixXd V_ = V;
        MatrixXi F_ = F;
        qslim::remove_duplicated_faces(V_, F_);
        return igl::is_edge_manifold(F_);
    }

    // self intersection for input and output test
    bool check_self_intersection(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree){
       // use this for custom self intersection check
       return self_intersection_check_full(V, F, tree);
    }

    // self intersection check for iteration (pre_collapse_callback)
    bool check_self_intersection(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                                 vector<int> &affected_triangle_indices, int removed_vertex_idx1,
                                 int removed_vertex_idx2){

        // test self intersection locally
        return self_intersection_check_local(V, F, tree, affected_triangle_indices, removed_vertex_idx1,
                                       removed_vertex_idx1);
    }

    // manifold test for input and output
    bool is_manifold(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree, bool useManifoldCheck) {
        // check edge_manifold
        clock_t start_edge, end_edge, start_intersect, end_intersect;
        //return true;

        // no need to check edge manifold since
        if (useManifoldCheck) {
            start_edge = clock();
            if (!check_edge_manifold(V, F)) {
                cout << "edge manifold test fail\n";
                return false;
            }
            end_edge = clock();
            //cout << "edge manifold test : " << (double) (end_edge - start_edge) / CLOCKS_PER_SEC << " sec\n";
        }
        //cout << "edge manifold test success" << endl;

        //check self-intersection
        start_intersect = clock();
        if (check_self_intersection(V, F, tree)) {
            // self intersection exist
            cout << "self-intersection test fail\n";
            return false;
        }
        end_intersect = clock();
        //cout << "self intersection test : " << (double) (end_intersect - start_intersect) / CLOCKS_PER_SEC << " sec\n";

        return true;
    }

    // manifold check for iteration (pre_collapse_callback)
    bool is_manifold(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                     vector<int> &affected_triangle_indices, int removed_vertex_idx1,
                     int removed_vertex_idx2,  bool useManifoldCheck){
        // check edge_manifold
        clock_t start_edge, end_edge, start_intersect, end_intersect;
        //return true;

        // no need to check edge manifold since
        if (useManifoldCheck) {
            start_edge = clock();
            if (!check_edge_manifold(V, F)) {
                cout << "edge manifold test fail\n";
                return false;
            }
            end_edge = clock();
            cout << "edge manifold test : " << (double) (end_edge - start_edge) / CLOCKS_PER_SEC << " sec\n";
        }
        //cout << "edge manifold test success" << endl;

        //check self-intersection
        start_intersect = clock();
        if (check_self_intersection(V, F, tree, affected_triangle_indices, removed_vertex_idx1,
                                    removed_vertex_idx2)) {
            // self intersection exist
            //cout << "self-intersection test fail\n";
            return false;
        }
        end_intersect = clock();
        //cout << "self intersection test : " << (double) (end_intersect - start_intersect) / CLOCKS_PER_SEC << " sec\n";

        return true;
    }
}

