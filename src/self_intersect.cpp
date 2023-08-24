//
// Created by seonghun on 8/23/23.
//

#include "self_intersect.h"

namespace qslim {
    bool
    tri_tri_intersection_check(const MatrixXd &V, const MatrixXi &F, unsigned int faceIdx1, unsigned int faceIdx2) {
        // Extract each vertex
        RowVector3d p1 = V.row(F(faceIdx1, 0));
        RowVector3d q1 = V.row(F(faceIdx1, 1));
        RowVector3d r1 = V.row(F(faceIdx1, 2));

        RowVector3d p2 = V.row(F(faceIdx2, 0));
        RowVector3d q2 = V.row(F(faceIdx2, 1));
        RowVector3d r2 = V.row(F(faceIdx2, 2));

        // whether two face is on same plane
        bool coplanar;
        // each end point of intersection line
        RowVector3d source, target;

        bool isIntersecting = igl::tri_tri_intersection_test_3d(p1, q1, r1, p2, q2, r2, coplanar, source, target);

        return isIntersecting;
    }

    bool self_intersection_test(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                                unordered_map<int, bool> &decimated_faces) {
        for (int faceIdx = 0; faceIdx < F.rows(); faceIdx++) {
            // No need to check decimated faces
            if (decimated_faces[faceIdx]) continue;
            // Compute AABB for current face
            Vector3d v1 = V.row(F(faceIdx, 0));
            Vector3d v2 = V.row(F(faceIdx, 1));
            Vector3d v3 = V.row(F(faceIdx, 2));

            Vector3d lowerBound = v1.cwiseMin(v2).cwiseMin(v3);
            Vector3d upperBound = v1.cwiseMax(v2).cwiseMax(v3);
            aabb::AABB currentAABB;

            for (int i = 0; i < 3; i++) {
                currentAABB.lowerBound.push_back(lowerBound[i]);
                currentAABB.upperBound.push_back(upperBound[i]);
            }

            // TODO: tree.query의 런타임이 너무 김
            // Use the tree's query function to find overlapping triangles
            std::vector<unsigned int> potentialIntersections = tree.query(faceIdx, currentAABB);
            unsigned int rootIdx = tree.getRootIdx();
            aabb::Node *node = tree.getNode(rootIdx);
            int cnt = 0;
            while (!node->isLeaf()) {
                node = tree.getNode(node->right);
                cnt++;
            }

            //cout << cnt << endl;

            // Check each candidate
/*            for (unsigned int candidateIdx: potentialIntersections) {
                if (tri_tri_intersection_check(V, F, faceIdx, candidateIdx))
                    return true;
            }*/
        }
        return false;
    }
}