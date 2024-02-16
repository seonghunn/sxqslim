//
// Created by seonghun on 8/22/23.
//

#ifndef QEM_MESHSIMPLIFY_H
#define QEM_MESHSIMPLIFY_H

#include "tree.h"
#include "AABB.hpp"
#include "remove_duplicated.h"
#include "quadratic.h"
#include "manifold.h"
#include "igl/opengl/glfw/Viewer.h"
#include <Eigen/Core>
#include <time.h>
#include <string>
#include <igl/min_heap.h>
#include <igl/writeOBJ.h>
#include <igl/collapse_edge.h>
#include <igl/circulation.h>
#include <unordered_set>
#include <igl/edge_flaps.h>
#include <iostream>
#include <igl/adjacency_list.h>

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
        // restore map for tree
        unordered_map<int, NodeSnapshot> restoreMap;

        clock_t start, end;
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

        // callback functions
        igl::decimate_cost_and_placement_callback cost_and_position_callback;
        igl::decimate_pre_collapse_callback pre_collapse;
        igl::decimate_post_collapse_callback post_collapse;

    public:
        MeshSimplify(MatrixXd &OV, MatrixXi &OF);

        // input manifold test
        bool input_manifold_test(MatrixXd &OV, MatrixXi &OF);

        // init member variable
        void init_member_variable(MatrixXd &OV, MatrixXi &OF);

        // init homogeneous surface normal
        static void init_normal_homo_per_face(MatrixXd &V, MatrixXi &F, MatrixXd &N_homo);

        // init qvalues
        void init_qValues(MatrixXd &V, MatrixXi &F, MatrixXd &N_homo);

        // init queue
        void init_queue();

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

        void process(double ratio);
    };
}


#endif //QEM_MESHSIMPLIFY_H
