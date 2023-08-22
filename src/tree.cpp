//
// Created by seonghun on 8/21/23.
//

#include "tree.h"

namespace qslim{
    void initializeTreeFromMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, aabb::Tree &tree) {
        for (int i = 0; i < F.rows(); ++i) {
            std::vector<double> lowerBound(3, std::numeric_limits<double>::max());
            std::vector<double> upperBound(3, -std::numeric_limits<double>::max());

            // 해당 face의 3개의 vertex에 대해 AABB를 계산합니다.
            for (int j = 0; j < 3; ++j) {
                Eigen::VectorXd vertex = V.row(F(i, j));
                for (int dim = 0; dim < 3; ++dim) {
                    lowerBound[dim] = std::min(lowerBound[dim], vertex[dim]);
                    upperBound[dim] = std::max(upperBound[dim], vertex[dim]);
                }
            }

            // 계산된 AABB를 Tree에 삽입합니다.
            tree.insertParticle(i, lowerBound, upperBound); // i는 face의 인덱스입니다.
        }
    }
}