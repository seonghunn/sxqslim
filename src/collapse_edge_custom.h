//
// Created by seonghun on 7/18/23.
//
#ifndef QEM_COLLAPSE_EDGE_CUSTOM_H
#define QEM_COLLAPSE_EDGE_CUSTOM_H
#pragma once

#include <igl/collapse_edge.h>
#include <Eigen/Core>
#include <igl/min_heap.h>
#include <vector>
#include "quadratic.h"

bool collapse_edge_custom(
        const igl::decimate_cost_and_placement_callback & cost_and_placement,
        Eigen::MatrixXd & V,
        Eigen::MatrixXi & F,
        Eigen::MatrixXi & E,
        Eigen::VectorXi & EMAP,
        Eigen::MatrixXi & EF,
        Eigen::MatrixXi & EI,
        igl::min_heap< std::tuple<double,int,int> > & Q,
        Eigen::VectorXi & EQ,
        Eigen::MatrixXd & C
        );

#endif