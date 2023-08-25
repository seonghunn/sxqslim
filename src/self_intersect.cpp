
//
// Created by seonghun on 8/23/23.
//

#include "self_intersect.h"
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
//#include <igl/tri_tri_intersection_test_3d.h>
//#include "aabb.h"

using namespace Eigen;
using namespace std;
namespace qslim {
    bool aabb_intersecting(const aabb::AABB &a, const aabb::AABB &b) {
        for (int i = 0; i < 3; i++) {
            if (a.lowerBound[i] > b.upperBound[i] || a.upperBound[i] < b.lowerBound[i]) {
                return false;
            }
        }
        return true;
    }

    aabb::AABB set_current_aabb(const MatrixXd &V, const MatrixXi &F, int faceIdx){
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

        return currentAABB;
    }

    template <typename Derived>
    bool adjacent_faces(const Eigen::MatrixBase<Derived>& A,
                        const Eigen::MatrixBase<Derived>& B)
    {
        for(int i=0;i<3;++i)
            for(int j=0;j<3;j++)
            {
                if(A(i)==B(j))
                    return true;
            }
        return false;
    }

    void dfs(aabb::Tree &tree, int nodeId, aabb::AABB &queryAABB, vector<int> &candidates) {
        const aabb::Node *node = tree.getNode(nodeId);
        if (node->isLeaf()) {
            candidates.push_back(node->particle);
            return;
        }
        if (aabb_intersecting(queryAABB, node->aabb)) {
            dfs(tree, node->left, queryAABB, candidates);
            dfs(tree, node->right, queryAABB, candidates);
        }
    }

    bool
    tri_tri_intersection_check(const MatrixXd &V, const MatrixXi &F, unsigned int faceIdx1, unsigned int faceIdx2) {
        RowVector3d p1 = V.row(F(faceIdx1, 0));
        RowVector3d q1 = V.row(F(faceIdx1, 1));
        RowVector3d r1 = V.row(F(faceIdx1, 2));

        RowVector3d p2 = V.row(F(faceIdx2, 0));
        RowVector3d q2 = V.row(F(faceIdx2, 1));
        RowVector3d r2 = V.row(F(faceIdx2, 2));

        bool coplanar;
        RowVector3d source, target;

        bool isIntersecting = igl::tri_tri_intersection_test_3d(p1, q1, r1, p2, q2, r2, coplanar, source, target);

        return isIntersecting;
    }

    bool self_intersection_check(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                                 unordered_map<int, bool> &decimated_faces) {
        for (int i = 0; i < F.rows(); i++) {
            // no need to check decimated faces
            if(decimated_faces[i]) continue;

            aabb::AABB queryAABB = set_current_aabb(V, F, i); // The AABB for triangle i

            vector<int> candidates;
            dfs(tree, tree.getRootIdx(), queryAABB, candidates);

            for (const int &candidateIdx: candidates) {
                if (tri_tri_intersection_check(V, F, i, candidateIdx)) {
                    if(candidateIdx == i)
                        continue;
                    if(adjacent_faces(F.row(i), F.row(candidateIdx)) )
                        continue;

                    return true;
                }
            }
        }
        return false;
    }
}
/*
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

*/
/*    bool self_intersection_test(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
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
*//*
*/
/*            for (unsigned int candidateIdx: potentialIntersections) {
                if (tri_tri_intersection_check(V, F, faceIdx, candidateIdx))
                    return true;
            }*//*
*/
/*
        }
        return false;
    }*//*


    bool aabb_intersecting(aabb::AABB a, aabb::AABB b){
        for (int i = 0; i < 3; i++) {
            if (a.lowerBound[i] > b.upperBound[i] || a.upperBound[i] < b.lowerBound[i]) {
                return false;
            }
        }
        return true;
    }

    void dfs(const aabb::AABB& queryAABB, aabb::Node* node,
                       aabb::Tree &tree, vector<int>& candidates) {
        if (node->isLeaf()) {
            candidates.push_back(node->particle);
            return;
        }

        if (aabb_intersecting(queryAABB, node->aabb)) {
            dfs(queryAABB, tree.getNode(node->left), tree, candidates);
            dfs(queryAABB, tree.getNode(node->right), tree, candidates);
        }
    }

    vector<int> region_intersection_test(const aabb::AABB &queryAABB, aabb::Tree &tree) {
        vector<int> candidates;
        dfs(queryAABB, tree.getNode(tree.getRootIdx()), tree, candidates);
        return candidates;
    }

    template <
            typename DerivedV,
            typename DerivedF,
            typename DerivedI>
    bool fast_find_self_intersections(
            const Eigen::MatrixBase<DerivedV>& V,
            const Eigen::MatrixBase<DerivedF>& F,
            Eigen::PlainObjectBase<DerivedI>& intersect,
            aabb::Tree &tree)
    {
        using Scalar=typename DerivedV::Scalar;
        using BBOX=Eigen::AlignedBox<Scalar,3>;
        //using AABBTree=igl::AABB<DerivedV,3>;
        //AABBTree tree;

        //tree.init(V,F);
        bool _intersects=false;

        intersect.resize(F.rows(),1);
        intersect.setConstant(0);

        for(int i=0; i<F.rows(); ++i)
        {
            if( intersect(i) ) continue;

            BBOX tri_box;

            for(int j=0;j<3;++j)
                tri_box.extend( V.row( F(i,j) ).transpose() );

            // find leaf nodes containing intersecting tri_box
            // need to declare full type to enable recursion
            std::function<bool(aabb::Tree &,int)> check_intersect =
                    [&](aabb::Tree &t,int d) -> bool
                    {
                        if(t.m_primitive != -1) //check for the actual intersection (is_leaf)
                        {
                            if(t.m_primitive==i) //itself
                                return false;
                            if(igl::internal::adjacent_faces(F.row(i), F.row(t.m_primitive)) )
                                return false;

                            bool coplanar=false;
                            Eigen::Matrix<Scalar,1,3,Eigen::RowMajor> edge1,edge2;

                            if(igl::tri_tri_intersection_test_3d(
                                    V.row(F(i,0)),V.row(F(i,1)),V.row(F(i,2)),
                                    V.row(F(t.m_primitive,0)),V.row(F(t.m_primitive,1)),V.row(F(t.m_primitive,2)),
                                    coplanar,
                                    edge1,edge2))
                            {
                                if(!coplanar)
                                {
                                    intersect(i)=1;
                                    intersect(t.m_primitive)=1;
                                    return true;
                                }
                            }
                        } else {
                            if(t.m_box.intersects(tri_box)) {
                                // need to check both subtrees
                                bool r1=check_intersect(*t.m_left ,d+1);
                                bool r2=check_intersect(*t.m_right,d+1);
                                return r1||r2;
                            }
                        }
                        return false;
                    };

            bool r=check_intersect(tree,0);
            _intersects = _intersects || r;
        }
        return _intersects;
    }
}*/
