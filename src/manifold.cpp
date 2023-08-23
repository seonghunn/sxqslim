//
// Created by seonghun on 7/23/23.
//

#include "manifold.h"

namespace PMP = CGAL::Polygon_mesh_processing;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;

namespace qslim{
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

    // hello AABB tree
    // Particle radius.
    double radius = 1.0;

// Set the particle position.
    std::vector<double> position({10, 10});

// Compute lower and upper AABB bounds.
    std::vector<double> lowerBound({position[0] - radius, position[1] - radius});
    std::vector<double> upperBound({position[0] + radius, position[1] + radius});

// Create the AABB.
    aabb::AABB aabb(lowerBound, upperBound);

    // self intersection using libigl
    bool check_self_intersection(const MatrixXd &V, const MatrixXi &F){
        //test aabb tree
        aabb::Tree tree;
        qslim::initializeTreeFromMesh(V, F, tree);

        Eigen::MatrixXi intersect, edges;

        //if it returns false, it means the mesh does not have self-intersections
       return igl::fast_find_self_intersections(V, F, intersect);
       //return true;
    }

    // self intersection using CGAL
/*    bool check_self_intersection(const MatrixXd &V, const MatrixXi &F){
        Mesh mesh;

        std::vector<Mesh::Vertex_index> vertices;

        // Convert Eigen matrix to CGAL Surface_mesh
        for(int i = 0; i < V.rows(); ++i) {
            vertices.push_back(mesh.add_vertex(Point_3(V(i,0), V(i,1), V(i,2))));
        }
        for(int i = 0; i < F.rows(); ++i) {
            mesh.add_face(vertices[F(i, 0)], vertices[F(i, 1)], vertices[F(i, 2)]);
        }

        // Check for self-intersections
        bool has_self_intersection = PMP::does_self_intersect(mesh, PMP::parameters::vertex_point_map(get(CGAL::vertex_point, mesh)));
        return has_self_intersection;
    }*/

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
            cout << "self-intersection test fail" << endl;
            return false;
        }
        end_intersect = clock();
        cout << "self intersect test : " << (double) (end_intersect - start_intersect) / CLOCKS_PER_SEC << " sec" << endl;
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

