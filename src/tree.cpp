//
// Created by seonghun on 8/21/23.
//

#include "tree.h"

namespace qslim{
    void initialize_tree_from_mesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, aabb::Tree &tree) {
        // for all faces
        for (int i = 0; i < F.rows(); ++i) {
            std::vector<double> lowerBound(3, std::numeric_limits<double>::max());
            std::vector<double> upperBound(3, -std::numeric_limits<double>::max());

            // Compute AABB for each vertex of face
            for (int j = 0; j < 3; j++) {
                Eigen::VectorXd vertex = V.row(F(i, j));
                for (int dim = 0; dim < 3; dim++) {
                    lowerBound[dim] = std::min(lowerBound[dim], vertex[dim]);
                    upperBound[dim] = std::max(upperBound[dim], vertex[dim]);
                }
            }

            // insert AABB
            tree.insertParticle(i, lowerBound, upperBound); // i is index of face
        }
    }

    //TODO: Debug this
    void update_tree_after_decimation(
            const Eigen::MatrixXd &V,
            const Eigen::MatrixXi &F,
            aabb::Tree &tree,
            int RV_idx1, int RV_idx2,
            int f1, int f2)
    {
        // 1. Identify affected triangles by checking RV and the provided face indices
        std::vector<int> affectedTriangleIndices = {f1, f2};
        for (int i = 0; i < F.rows(); i++) {
            if (F(i, 0) == RV_idx1 || F(i, 1) == RV_idx1 || F(i, 2) == RV_idx1 ||
                F(i, 0) == RV_idx2 || F(i, 1) == RV_idx2 || F(i, 2) == RV_idx2)
            {
                affectedTriangleIndices.push_back(i);
            }
        }

        // Remove duplicates from the list
        std::sort(affectedTriangleIndices.begin(), affectedTriangleIndices.end());
        affectedTriangleIndices.erase(std::unique(affectedTriangleIndices.begin(), affectedTriangleIndices.end()), affectedTriangleIndices.end());

        // 2. Recompute AABBs for affected triangles and update the tree
        for (int triangleIdx : affectedTriangleIndices) {
            Eigen::Vector3d v1 = V.row(F(triangleIdx, 0));
            Eigen::Vector3d v2 = V.row(F(triangleIdx, 1));
            Eigen::Vector3d v3 = V.row(F(triangleIdx, 2));

            // Compute new AABB
            Eigen::Vector3d lowerBound = v1.cwiseMin(v2).cwiseMin(v3);
            Eigen::Vector3d upperBound = v1.cwiseMax(v2).cwiseMax(v3);

            std::vector<double> lowerBoundVec = {lowerBound[0], lowerBound[1], lowerBound[2]};
            std::vector<double> upperBoundVec = {upperBound[0], upperBound[1], upperBound[2]};

            // 3. Update the tree
            tree.updateParticle(triangleIdx, lowerBoundVec, upperBoundVec);
        }
        // update test // update test success, but they didn't update ancestor
        //TODO: Recursively update the ancestor
        std::vector<double> lowerBoundVec = {-10, -10, -10};
        std::vector<double> upperBoundVec = {10, 10, 10};
        // find tree.nodes[13]
        tree.updateParticle(7, lowerBoundVec, upperBoundVec);
        // 4. Remove the faces that have been collapsed (assuming they are no longer in the mesh)
        tree.removeParticle(f1);
        tree.removeParticle(f2);
    }

    void update_ancestors(){

    }
}