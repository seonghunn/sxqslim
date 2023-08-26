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

    bool update_ancestors(aabb::Tree &tree, unsigned int triangleIdx, unsigned int dim,
                          std::unordered_map<int, NodeSnapshot> &restoreMap) {
        unsigned int nodeIdx = tree.getParticleNodeMapping(triangleIdx);
        aabb::Node *node = tree.getNode(nodeIdx);
        aabb::Node *parentNode;
        // all triangle must be in leaf node
        if (!node->isLeaf()) return false;
        // if node is parent, no need to be updated
        if (node->parent == NULL_NODE) return false;

        // Compare AABB with parent and current node, update it
        // TODO: get first node index using particleMap
        while (node->parent != NULL_NODE) {
            unsigned int parentIdx = node->parent;
            parentNode = tree.getNode(parentIdx);
            //unsigned int updatingParticleIdx = parentNode->particle;
            std::vector<double> currentLowerBoundVec = node->aabb.lowerBound;
            std::vector<double> currentUpperBoundVec = node->aabb.upperBound;

            std::vector<double> parentLowerBoundVec = parentNode->aabb.lowerBound;
            std::vector<double> parentUpperBoundVec = parentNode->aabb.upperBound;

            std::vector<double> lowerBoundVec(dim);
            std::vector<double> upperBoundVec(dim);
            for (int i = 0; i < dim; i++) {
                lowerBoundVec[i] = currentLowerBoundVec[i] < parentUpperBoundVec[i] ?
                                   currentLowerBoundVec[i] : parentLowerBoundVec[i];
                upperBoundVec[i] = currentUpperBoundVec[i] > parentUpperBoundVec[i] ?
                                   currentUpperBoundVec[i] : parentUpperBoundVec[i];
            }
            // save it to restore map
            NodeSnapshot ns;
            ns.isDeleted = false;
            ns.particleIdx = parentIdx;
            ns.lowerBound = parentNode->aabb.lowerBound;
            ns.upperBound = parentNode->aabb.upperBound;
            restoreMap[parentIdx] = ns;

            //tree.updateParticle(updatingParticleIdx, lowerBoundVec, upperBoundVec);
            parentNode->aabb.lowerBound = lowerBoundVec;
            parentNode->aabb.upperBound = upperBoundVec;
            parentNode->aabb.centre = parentNode->aabb.computeCentre();
            parentNode->aabb.surfaceArea = parentNode->aabb.computeSurfaceArea();
            node = parentNode;
        }
        return true;
    }

    bool update_tree_after_decimation(
            const Eigen::MatrixXd &V,
            const Eigen::MatrixXi &F,
            aabb::Tree &tree,
            int RV_idx1, int RV_idx2,
            int f1, int f2, std::unordered_map<int, bool> &decimatedFaces,
            std::unordered_map<int, std::vector<int>> &affectedTriangleIndices,
            std::unordered_map<int, NodeSnapshot> &restoreMap) {
        // 1. Identify affected triangles by checking RV and the provided face indices
        //std::vector<int> affectedTriangleIndices = {f1, f2};
/*        std::vector<int> affectedTriangleIndices;
        for (int i = 0; i < F.rows(); i++) {
            // exclude decimated faces (duplicated reference)
            if (decimatedFaces[i])
                continue;
            if (F(i, 0) == RV_idx1 || F(i, 1) == RV_idx1 || F(i, 2) == RV_idx1 ||
                F(i, 0) == RV_idx2 || F(i, 1) == RV_idx2 || F(i, 2) == RV_idx2) {
                affectedTriangleIndices.push_back(i);
            }
        }듯

        // Remove duplicates from the list
        std::sort(affectedTriangleIndices.begin(), affectedTriangleIndices.end());
        affectedTriangleIndices.erase(std::unique(affectedTriangleIndices.begin(), affectedTriangleIndices.end()),
                                      affectedTriangleIndices.end());*/

        // 2. Recompute AABBs for affected triangles and update the tree
        std::set<int> affectedTriangleSet(affectedTriangleIndices[RV_idx1].begin(),
                                          affectedTriangleIndices[RV_idx1].end());
        affectedTriangleSet.insert(affectedTriangleIndices[RV_idx2].begin(), affectedTriangleIndices[RV_idx2].end());

        for (int triangleIdx: affectedTriangleSet) {
            if (decimatedFaces[triangleIdx])
                continue;
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
            NodeSnapshot ns;
            ns.isDeleted = false;
            ns.particleIdx = triangleIdx;
            ns.lowerBound = lowerBoundVec;
            ns.upperBound = upperBoundVec;
            restoreMap[triangleIdx] = ns;
            qslim::update_ancestors(tree, triangleIdx, 3, restoreMap);
        }
/*        // update test // update test success, but they didn't update ancestor
        std::vector<double> lowerBoundVec = {-10, -10, -10};
        std::vector<double> upperBoundVec = {10, 10, 10};
        // find tree.nodes[13]
        tree.updateParticle(7, lowerBoundVec, upperBoundVec);
        qslim::update_ancestors(tree, 7, 3);*/
        // 4. Remove the faces that have been collapsed (assuming they are no longer in the mesh)
        aabb::Node *tmp = tree.getNode(f1);
        NodeSnapshot tmpNs;
        tmpNs.isDeleted = false;
        tmpNs.particleIdx = tmp->particle;
        tmpNs.lowerBound = tmp->aabb.lowerBound;
        tmpNs.upperBound = tmp->aabb.upperBound;
        restoreMap[f1] = tmpNs;
        tree.removeParticle(f1);

        tmp = tree.getNode(f2);
        tmpNs.isDeleted = false;
        tmpNs.particleIdx = tmp->particle;
        tmpNs.lowerBound = tmp->aabb.lowerBound;
        tmpNs.upperBound = tmp->aabb.upperBound;
        restoreMap[f2] = tmpNs;
        tree.removeParticle(f2);

        return true;
    }
}