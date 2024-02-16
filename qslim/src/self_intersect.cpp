
//
// Created by seonghun on 8/23/23.
//

#include "self_intersect.h"

using namespace Eigen;
using namespace std;
namespace qslim
{
    bool aabb_intersecting(const aabb::AABB &a, const aabb::AABB &b)
    {
        for (int i = 0; i < 3; i++)
        {
            if (a.lowerBound[i] > b.upperBound[i] || a.upperBound[i] < b.lowerBound[i])
            {
                return false;
            }
        }
        return true;
    }

    aabb::AABB set_current_aabb(const MatrixXd &V, const MatrixXi &F, int faceIdx)
    {
        Vector3d v1 = V.row(F(faceIdx, 0));
        Vector3d v2 = V.row(F(faceIdx, 1));
        Vector3d v3 = V.row(F(faceIdx, 2));

        Vector3d lowerBound = v1.cwiseMin(v2).cwiseMin(v3);
        Vector3d upperBound = v1.cwiseMax(v2).cwiseMax(v3);
        aabb::AABB currentAABB;

        for (int i = 0; i < 3; i++)
        {
            currentAABB.lowerBound.push_back(lowerBound[i]);
            currentAABB.upperBound.push_back(upperBound[i]);
        }

        return currentAABB;
    }

    template <typename Derived>
    int get_num_of_shared_vertex(const Eigen::MatrixBase<Derived> &A,
                                 const Eigen::MatrixBase<Derived> &B)
    {
        int num = 0;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; j++)
            {
                if (A(i) == B(j))
                    num++;
            }
        return num;
    }

    void dfs(aabb::Tree &tree, int nodeId, aabb::AABB &queryAABB, vector<int> &candidates)
    {
        const aabb::Node *node = tree.getNode(nodeId);
        if (node->isLeaf())
        {
            candidates.push_back(node->particle);
            return;
        }
        if (aabb_intersecting(queryAABB, node->aabb))
        {
            dfs(tree, node->left, queryAABB, candidates);
            dfs(tree, node->right, queryAABB, candidates);
        }
    }

    bool
    tri_tri_intersection_check(const MatrixXd &V, const MatrixXi &F, unsigned int faceIdx1, unsigned int faceIdx2,
                               int shared_vertex)
    {
        RowVector3d p1 = V.row(F(faceIdx1, 0));
        RowVector3d q1 = V.row(F(faceIdx1, 1));
        RowVector3d r1 = V.row(F(faceIdx1, 2));

        RowVector3d p2 = V.row(F(faceIdx2, 0));
        RowVector3d q2 = V.row(F(faceIdx2, 1));
        RowVector3d r2 = V.row(F(faceIdx2, 2));

        // avg normalize
        RowVector3d mean = (p1 + q1 + r1 + p2 + q2 + r2) / 6.0;
        mean = mean * 10;
        p1 = p1.array() / mean.array();
        q1 = q1.array() / mean.array();
        r1 = r1.array() / mean.array();
        p2 = p2.array() / mean.array();
        q2 = q2.array() / mean.array();
        r2 = r2.array() / mean.array();

        bool coplanar;
        RowVector3d source, target;
        float r_i1[3];
        float r_i2[3];
        bool isIntersecting = isect_tri_tri_v3(p1, q1, r1, p2, q2, r2, r_i1, r_i2);
        copy_v3_v3_rowVec3d_float(source, r_i1);
        copy_v3_v3_rowVec3d_float(target, r_i2);
        double threshold = 1e-3;
        bool isClose = ((source - target).array().abs() <= threshold).all();
        // if the intersection line is cloesed to shared vertex
        if (isIntersecting && shared_vertex == 1 && isClose)
        {
            // not intersect
            return false;
        }
/*        if (isIntersecting)
        {
            cout << "faceIdx 1 :" << faceIdx1 << "\n";
            cout << "faceIdx 2 : " << faceIdx2 << "\n";
            // cout << "F1\n";
            cout << "v " << p1 << endl;
            cout << "v " << q1 << endl;
            cout << "v " << r1 << endl;
            // cout << "F2" << endl;
            cout << "v " << p2 << endl;
            cout << "v " << q2 << endl;
            cout << "v " << r2 << endl;
            cout << "f 1 2 3" << endl;
            cout << "f 4 5 6" << endl;
            cout << "source - target" << endl;
            cout << "source : " << source << endl;
            cout << "target : " << target << endl;
        }*/
        return isIntersecting;
    }

    bool self_intersection_check_full(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree)
    {
        for (int i = 0; i < F.rows(); i++)
        {
            // no need to check decimated faces
            if (F(i, 0) == 0 && F(i, 1) == 0 && F(i, 2) == 0)
            {
                continue;
            }

            aabb::AABB queryAABB = set_current_aabb(V, F, i); // The AABB for triangle i

            vector<int> candidates;
            dfs(tree, tree.getRootIdx(), queryAABB, candidates);

            for (const int &candidateIdx : candidates)
            {
                // if the candidate triangle is same as input
                if (candidateIdx == i)
                    continue;
                // if the candidate triangle is decimated faces
                if (F(candidateIdx, 0) == 0 && F(candidateIdx, 1) == 0 && F(candidateIdx, 2) == 0)
                    continue;
                // check actual tri-tri intersect
                int shared_vertex = get_num_of_shared_vertex(F.row(i), F.row(candidateIdx));
                if (shared_vertex == 2)
                    continue;
                if (tri_tri_intersection_check(V, F, i, candidateIdx, shared_vertex))
                {
                    return true;
                }
            }
        }
        return false;
    }

    void get_one_ring_neigh(
        const int e,
        const Eigen::MatrixXi &F,
        const Eigen::VectorXi &EMAP,
        const Eigen::MatrixXi &EF,
        const Eigen::MatrixXi &EI,
        std::vector<int> &one_ring_faces)
    {
        std::vector<int> /*Nse,*/ Nsf, Nsv;
        igl::circulation(e, true, F, EMAP, EF, EI, /*Nse,*/ Nsv, Nsf);
        std::vector<int> /*Nde,*/ Ndf, Ndv;
        igl::circulation(e, false, F, EMAP, EF, EI, /*Nde,*/ Ndv, Ndf);

        std::vector<int> Nf;
        Nf.reserve(Nsf.size() + Ndf.size()); // preallocate memory
        Nf.insert(Nf.end(), Nsf.begin(), Nsf.end());
        Nf.insert(Nf.end(), Ndf.begin(), Ndf.end());

        one_ring_faces = Nf;
    }

    bool self_intersection_check_local(const MatrixXd &V, const MatrixXi &F, aabb::Tree &tree,
                                       vector<int> &affected_triangle_indices, int removed_vertex_idx1,
                                       int removed_vertex_idx2)
    {
        for (int i : affected_triangle_indices)
        {
            // no need to check decimated faces
            if (F(i, 0) == 0 && F(i, 1) == 0 && F(i, 2) == 0)
            {
                continue;
            }

            aabb::AABB queryAABB = set_current_aabb(V, F, i); // The AABB for triangle i

            vector<int> candidates;
            dfs(tree, tree.getRootIdx(), queryAABB, candidates);

            for (const int &candidateIdx : candidates)
            {
                // if the candidate triangle is same as input
                if (candidateIdx == i)
                    continue;
                // if the candidate triangle is decimated faces
                if (F(candidateIdx, 0) == 0 && F(candidateIdx, 1) == 0 && F(candidateIdx, 2) == 0)
                    continue;
                // check actual tri-tri intersect
                int shared_vertex = get_num_of_shared_vertex(F.row(i), F.row(candidateIdx));
                // edge sharing cannot cause self intersect
                if (shared_vertex == 2)
                    continue;
                if (tri_tri_intersection_check(V, F, i, candidateIdx, shared_vertex))
                {
                    return true;
                }
            }
        }
        return false;
    }

}