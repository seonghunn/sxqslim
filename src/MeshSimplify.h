//
// Created by seonghun on 8/22/23.
//

#ifndef QEM_MESHSIMPLIFY_H
#define QEM_MESHSIMPLIFY_H

#include "tree.h"
#include "../include/AABB.hpp"
#include "helper.h"
#include "quadratic.h"
#include "manifold.h"
#include "igl/opengl/glfw/Viewer.h"
#include <Eigen/Core>
#include <time.h>
#include <string>
#include <igl/min_heap.h>
#include <igl/writeOBJ.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <iostream>

using namespace std;
using namespace Eigen;
namespace qslim{
    class MeshSimplify{
    private:
        // AABB tree
        aabb::Tree tree;
        // Min heap (priority queue) for greedy decimation
        igl::min_heap< std::tuple<double,int,int> > queue;
        // Q values table using surface normal
        std::vector<Eigen::Matrix4d> qValues;
        // viewer
        igl::opengl::glfw::Viewer viewer;
        // candidate for removing vertices
        index_of_removed_vertices RV;

        // mesh data for each timestamps
        MatrixXd V, C, N_homo;
        MatrixXi F, E, EF, EI;
        VectorXi EMAP, EQ;

        // input data
        MatrixXd OV;
        MatrixXi OF;
        int num_input_vertices;
        int num_input_faces;
        double ratio;

        int num_collapsed;
        int num_failed;
        int stopping_condition;
        // hash map for deleted faces
        std::unordered_map<int, bool> decimated_faces;

        // hash map for affected_triangles (vertex -> face mapping)
        //TODO: update affected triangle indices every time (vertices are merged into one)
        // 두개가 생기는 거기 때문에 두 개의 원래 vertex의 index를 똑같이 두개의 합으로 업데이트 해주면 될듯
        std::unordered_map<int, vector<int>> affected_triangle_indices;

        string output_filename;

        // callback functions
        igl::decimate_cost_and_placement_callback cost_and_position_callback;
        igl::decimate_pre_collapse_callback pre_collapse;
        igl::decimate_post_collapse_callback post_collapse;

    public:
        MeshSimplify(MatrixXd &OV, MatrixXi &OF, double ratio, string output_filename);

        // input manifold test
        bool input_manifold_test(MatrixXd &OV, MatrixXi &OF);

        // init member variable
        void init_member_variable(MatrixXd &OV, MatrixXi &OF, double ratio, string output_filename);

        // init affect_triangles table
        void init_affect_triangles(MatrixXd &OV, MatrixXi &OF);

        // init homogeneous surface normal
        static void init_normal_homo_per_face(MatrixXd &V, MatrixXi &F, MatrixXd &N_homo);

        // init qvalues
        void init_qValues(MatrixXd &V, MatrixXi &F, MatrixXd &N_homo);

        // init queue
        void init_queue(MatrixXd &OV, MatrixXi &OF);

        // init callback
        void init_callback();

        // getter
        MatrixXd get_vertices();

        MatrixXi get_faces();

        igl::decimate_cost_and_placement_callback get_cost_and_position_callback();
        igl::decimate_pre_collapse_callback get_pre_collapse_callback();
        igl::decimate_post_collapse_callback get_post_collapse_callback();

        // find cost and optimal position using each qvalues
        void cost_and_position(
                const int e,
                const Eigen::MatrixXd & V,
                const Eigen::MatrixXi & /*F*/,
                const Eigen::MatrixXi & E,
                const Eigen::VectorXi & /*EMAP*/,
                const Eigen::MatrixXi & /*EF*/,
                const Eigen::MatrixXi & /*EI*/,
                double & cost,
                Eigen::RowVectorXd & p);

        bool process();
    };
}


#endif //QEM_MESHSIMPLIFY_H
