//
// Created by seonghun on 7/23/23.
//

#include "manifold.h"

namespace PMP = CGAL::Polygon_mesh_processing;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;

namespace qslim{
    bool check_edge_manifold(const MatrixXd &V, const MatrixXi &F){
        MatrixXd V_ = V;
        MatrixXi F_ = F;
        qslim::remove_duplicated_faces(V_, F_);
        return igl::is_edge_manifold(F_);
    }

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

    // self intersection for input and output test
    bool check_self_intersection(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree, unordered_map<int, bool> &decimated_faces){
/*        //test aabb tree
        aabb::Tree tree;
        qslim::initialize_tree_from_mesh(V, F, tree);
*/

        //if it returns false, it means the mesh does not have self-intersections
        // use this for libigl
/*        Eigen::MatrixXi intersect, edges;
        Eigen::MatrixXd V_ = V;
        Eigen::MatrixXi F_ = F;
        remove_duplicated_faces(V_, F_);
        return igl::fast_find_self_intersections(V_, F_, intersect);*/

        //return true;
       // use this for custom self intersection check
       return self_intersection_check_full(V, F, tree, decimated_faces);
    }

    // self intersection check for iteration (pre_collapse_callback)
    bool check_self_intersection(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                                 unordered_map<int, bool> &decimated_faces,
                                 unordered_map<int, vector<int>> &affected_triangle_indices, int removed_vertex_idx1,
                                 int removed_vertex_idx2){

        // test self intersection locally
        return self_intersection_check_local(V, F, tree, decimated_faces, affected_triangle_indices, removed_vertex_idx1,
                                       removed_vertex_idx1);

        // test all self intersection
        //return self_intersection_check_full(V, F, tree, decimated_faces);

        // use this for libigl
/*        Eigen::MatrixXi intersect, edges;
        Eigen::MatrixXd V_ = V;
        Eigen::MatrixXi F_ = F;
        remove_duplicated_faces(V_, F_);
        return igl::fast_find_self_intersections(V_, F_, intersect);*/
    }

    // manifold test for input and output
    bool is_manifold(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                     unordered_map<int, bool> &decimated_faces, bool useManifoldCheck) {
        // check edge_manifold
        clock_t start_edge, end_edge, start_intersect, end_intersect;

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
        if (check_self_intersection(V, F, tree, decimated_faces)) {
            // self intersection exist
            cout << "self-intersection test fail\n";
            return false;
        }
        end_intersect = clock();
        cout << "self intersection test : " << (double) (end_intersect - start_intersect) / CLOCKS_PER_SEC << " sec\n";

/*        // check orientation
        if(!qslim::check_mesh_orientation(V, F)){
            cout << "orientation test fail" << endl;
            return false;
        }

        cout << "orientation test success" << endl;*//*


*/

        return true;
    }

    // manifold check for iteration (pre_collapse_callback)
    bool is_manifold(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                     unordered_map<int, bool> &decimated_faces,
                     unordered_map<int, vector<int>> &affected_triangle_indices, int removed_vertex_idx1,
                     int removed_vertex_idx2,  bool useManifoldCheck){
        // check edge_manifold
        clock_t start_edge, end_edge, start_intersect, end_intersect;

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
        if (check_self_intersection(V, F, tree, decimated_faces, affected_triangle_indices, removed_vertex_idx1,
                                    removed_vertex_idx2)) {
            // self intersection exist
            cout << "self-intersection test fail\n";
            return false;
        }
        end_intersect = clock();
        cout << "self intersection test : " << (double) (end_intersect - start_intersect) / CLOCKS_PER_SEC << " sec\n";
        //cout << "self-intersection test success" << endl;

/*        // check orientation
        if(!qslim::check_mesh_orientation(V, F)){
            cout << "orientation test fail" << endl;
            return false;
        }

        cout << "orientation test success" << endl;*/


        return true;
    }
}

