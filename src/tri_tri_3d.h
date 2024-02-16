//
// Created by seonghun on 2/8/24.
//

// based on blender

#ifndef QSLIM_TRI_TRI_3D_H
#define QSLIM_TRI_TRI_3D_H

#include <Eigen/Core>

bool isect_tri_tri_v3(const Eigen::RowVector3d &p1, const Eigen::RowVector3d &q1, const Eigen::RowVector3d &r1,
                      const Eigen::RowVector3d &p2, const Eigen::RowVector3d &q2, const Eigen::RowVector3d &r2,
                      float r_i1[3], float r_i2[3]);

void copy_v3_v3_rowVec3d_float(Eigen::RowVector3d &r, float a[3]);
#endif //QSLIM_TRI_TRI_3D_H
