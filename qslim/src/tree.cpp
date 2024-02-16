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

    // TODO: fix this (inifinte loop)
    bool update_tree_after_decimation(
            const Eigen::MatrixXd &V,
            const Eigen::MatrixXi &F,
            aabb::Tree &tree,
            int RV_idx1, int RV_idx2,
            vector<int> &combinedAffectedTriangleIndices) {
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

        // Recompute AABBs for affected triangles and update the tree
        for (int triangleIdx: combinedAffectedTriangleIndices) {
            if (F(triangleIdx,0)== 0 && F(triangleIdx, 1)==0 && F(triangleIdx, 2)==0){
                //remove
                //tree.removeParticle(triangleIdx);
                continue;
            }
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
            // TODO: restore it if need ------
            //tree.removeParticle(triangleIdx);
            //tree.insertParticle(triangleIdx, lowerBoundVec, upperBoundVec);
            //---------------------
            /*            NodeSnapshot ns;
            ns.isDeleted = false;
            ns.particleIdx = triangleIdx;
            ns.lowerBound = lowerBoundVec;
            ns.upperBound = upperBoundVec;
            restoreMap[triangleIdx] = ns;*/

            // no need to update tree again
            //qslim::update_ancestors(tree, triangleIdx, 3);
        }
/*        tree.removeParticle(f1);
        tree.removeParticle(f2);*/


        return true;
    }

    void getCombinedAffectedTriangleIndices(unordered_map<int, vector<int>> &affectedTriangleIndices,
                                            unordered_map<int, bool> &decimatedFaces, int RV_idx1, int RV_idx2,
                                            vector<int> &combinedAffectedTriangleIndices) {
        // update affected triangle indices (list)
        // combined : set of affected triangles except decimated triangles (faces)
        std::vector<int> combined;
        for (int faceIdx: affectedTriangleIndices[RV_idx1]) {
            // if face in the list is not decimated yet
            if (!decimatedFaces[faceIdx])
                combined.push_back(faceIdx);
        }
        for (int faceIdx: affectedTriangleIndices[RV_idx2]) {
            if (!decimatedFaces[faceIdx])
                combined.push_back(faceIdx);
        }
        // Remove duplicate
        std::sort(combined.begin(), combined.end());
        combined.erase(std::unique(combined.begin(), combined.end()), combined.end());

        // Two vertex are merged into new position -> assign same value for each list
/*        affectedTriangleIndices[RV_idx1] = combined;
        affectedTriangleIndices[RV_idx2] = combined;
*/
        combinedAffectedTriangleIndices = combined;
    }

    void
    takeNodeSnapShot(vector<int> &combinedAffectedTriangleIndices, aabb::Tree &tree,
                     unordered_map<int, NodeSnapshot> &nodeRestoreMap) {
        // for modified faces
        for (int triangleIdx: combinedAffectedTriangleIndices) {
            NodeSnapshot ns;
            int nodeIdx = tree.getParticleNodeMapping(triangleIdx);
            aabb::Node *node = tree.getNode(nodeIdx);
            ns.lowerBound = node->aabb.lowerBound;
            ns.upperBound = node->aabb.upperBound;
            ns.isDeleted = false;
            ns.particleIdx = triangleIdx;
            nodeRestoreMap.insert(make_pair(triangleIdx, ns));
        }

/*        // for deleted faces
        NodeSnapshot ns1;
        int nodeIdx1 = tree.getParticleNodeMapping(removedFaceIdx1);
        aabb::Node *node1 = tree.getNode(nodeIdx1);
        ns1.lowerBound = node1->aabb.lowerBound;
        ns1.upperBound = node1->aabb.upperBound;
        ns1.isDeleted = true;
        ns1.particleIdx = removedFaceIdx1;
        nodeRestoreMap.insert(make_pair(removedFaceIdx1, ns1));

        NodeSnapshot ns2;
        int nodeIdx2 = tree.getParticleNodeMapping(removedFaceIdx2);
        aabb::Node *node2 = tree.getNode(nodeIdx2);
        ns2.lowerBound = node2->aabb.lowerBound;
        ns2.upperBound = node2->aabb.upperBound;
        ns2.isDeleted = true;
        ns2.particleIdx = removedFaceIdx2;
        nodeRestoreMap.insert(make_pair(removedFaceIdx2, ns2));*/
    }

    void restoreTree(vector<int> combinedAffectedTriangleIndices, unordered_map<int, NodeSnapshot> &restoreMap,
                     aabb::Tree &tree, int removedFaceIdx1,
                     int removedFaceIdx2) {

        // deal with affected faces (not decimated)
        for (int triangleIdx: combinedAffectedTriangleIndices) {
            //this->tree.removeParticle(triangleIdx);
            // if face is deleted at collapsing step, just update it
            if (!restoreMap[triangleIdx].isDeleted) {
                qslim::NodeSnapshot ns = restoreMap[triangleIdx];
                //tree.removeParticle(triangleIdx);
                //tree.insertParticle(triangleIdx, ns.lowerBound, ns.upperBound);
                tree.updateParticle(triangleIdx, ns.lowerBound, ns.upperBound, true);
            }
        }
        // deal with decimated faces
        qslim::NodeSnapshot ns1 = restoreMap[removedFaceIdx1];
        qslim::NodeSnapshot ns2 = restoreMap[removedFaceIdx2];
        // TODO: restore this
        //tree.insertParticle(removedFaceIdx1, ns1.lowerBound, ns1.upperBound);
        //tree.insertParticle(removedFaceIdx2, ns2.lowerBound, ns2.upperBound);
    }
}

/*    bool update_ancestors(aabb::Tree &tree, unsigned int triangleIdx, unsigned int dim) {
        aabb::Node *node = tree.getNode(triangleIdx);
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
            parentNode->aabb.lowerBound = lowerBoundVec;
            parentNode->aabb.upperBound = upperBoundVec;
            parentNode->aabb.centre = parentNode->aabb.computeCentre();
            parentNode->aabb.surfaceArea = parentNode->aabb.computeSurfaceArea();
            node = parentNode;
        }
        return true;
    }*/
