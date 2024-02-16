//
// Created by seonghun on 8/21/23.
//

#ifndef QEM_TREE_H
#define QEM_TREE_H

#include "AABB.hpp"
#include "remove_duplicated.h"
#include <Eigen/Core>

using namespace Eigen;
using namespace std;
namespace qslim{
    // structure for restoring node data
    struct NodeSnapshot{
        bool isDeleted;
        int particleIdx;
        vector<double> lowerBound;
        vector<double> upperBound;
    };

    //init AABB tree
    void initialize_tree_from_mesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, aabb::Tree &tree);

    //update ancestors of leaf
    bool update_ancestors(aabb::Tree &tree, unsigned int triangleIdx, unsigned int dim);

    /*!\return
     *
     * @param V vertices
     * @param F faces
     * @param tree aabb tree
     * @param RV_idx1 removed vertex index 1
     * @param RV_idx2 removed vertex index 1
     * @param f1 removed face index 1
     * @param f2 removed face index 1
     * @param affectedTriangleIndices affected triangle indices hash map (key : vertex, value : vector list of indices of affected faces)
     * @return is updating done without error
     */
    //update tree after decimation
    bool update_tree_after_decimation(
            const Eigen::MatrixXd &V,
            const Eigen::MatrixXi &F,
            aabb::Tree &tree,
            int RV_idx1, int RV_idx2,
            vector<int> &affectedTriangleIndices);


    // take a node restore map to undo tree update if self intersection check fails
    /*!
     * @param combinedAffectedTriangleIndices combined list (1-ring neighborhood) of affected triangle indices after collapsing edge e
     * @param tree
     * @param removedFaceIdx1
     * @param removedFaceIdx2
     * @param nodeRestoreMap
     */
    void
    takeNodeSnapShot(vector<int> &combinedAffectedTriangleIndices, aabb::Tree &tree,
                     unordered_map<int, NodeSnapshot> &nodeRestoreMap);

    /*!
     *
     * @param combinedAffectedTriangleIndices
     * @param restoreMap
     * @param tree
     * @param removedFaceIdx1
     * @param removedFaceIdx2
     */

    // restore tree
    void restoreTree(vector<int> combinedAffectedTriangleIndices, unordered_map<int, NodeSnapshot> &restoreMap,
                     aabb::Tree &tree, int removedFaceIdx1,
                     int removedFaceIdx2);
}


#endif //QEM_TREE_H
