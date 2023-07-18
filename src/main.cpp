#include "igl/opengl/glfw/Viewer.h"
#include "igl/decimate.h"
#include "igl/per_face_normals.h"
#include "igl/per_vertex_attribute_smoothing.h"
#include "igl/edge_flaps.h"
#include "igl/adjacency_list.h"
#include <unordered_map>
#include <queue>
#include <iostream>
#include <vector>
#include <set>
using namespace std;

// Custom structure for priority queue
struct Pair{
    int v1, v2;
    Eigen::Vector4d target;
    double cost;

    // Define the comparison operator for the priority queue (min heap)
    bool operator<(const Pair &other) const {
        // larger value of cost is treated as less using operator overloading
        return cost > other.cost;
    }
};

int main(int argc, char *argv[])
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    // man.obj : non-manifold mesh
    //igl::readOBJ("../model/input/man.obj",V,F);
    igl::readOBJ("../model/input/cube.obj", V, F);

    cout << "Number of Input vertices : "<<V.rows()<<endl;
    cout << "Number of Input faces : "<<F.rows()<<endl;
    Eigen::MatrixXd V_out = Eigen::MatrixXd::Zero(V.rows(), 3);
    Eigen::MatrixXi F_out;
    Eigen::VectorXi J;
    Eigen::VectorXi I;
    Eigen::MatrixXd N;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi uE, EF, EI;
    igl::per_face_normals(V, F, N);
    igl::edge_flaps(F, uE, EMAP, EF, EI);
    Eigen::MatrixXd N_homo(F.rows(), 4);

    // turns N into N_homo (3D homogeneous coordinate)
    for (int i = 0; i < F.rows(); i++) {
        Eigen::RowVector3d n = N.row(i);
        double d = -n.dot(V.row(F(i, 0)));
        N_homo.row(i) << n, d;
    }

    // Initial Quadrics for each vertex
    vector<Eigen::Matrix4d> quadrics(V.rows(), Eigen::Matrix4d::Zero());
    Eigen::MatrixXd Q;
    for (int i = 0; i < F.rows(); i++) {
        Eigen::Vector4d p(N_homo(i, 0), N_homo(i, 1), N_homo(i, 2), N_homo(i, 3));
        Eigen::Matrix4d q = p * p.transpose();
        // Add q for each 3 vertex in face, addition for summing q for all adjacent planes
        for (int j = 0; j < 3; j++) {
            quadrics[F(i,j)] += q;
        }
    }

    // Initialize the priority queue
    priority_queue<Pair> pq;
    // Initialize the valid pair
    vector<set<int>> validPairs(V.rows());

    // Calculate Adjacency list
    vector<vector<int>> adjacencyList;
    igl::adjacency_list(F, adjacencyList);

    // Calculate the cost for each pair and add it to the priority queue
    //TODO: Remove the redundancy (i,j) - (j,i)
    for (int i = 0; i < adjacencyList.size(); i++) {
        for (int j = 0; j < adjacencyList[i].size(); j++) {
            // index of each vertex
            int v1 = i;
            int v2 = adjacencyList[i][j];

            // Calculate the optimal contraction
            Eigen::Matrix4d Q = quadrics[v1] + quadrics[v2];
            //TODO: check whether Q is invertable and if it is singular, assign midpoint or each vertex

            // Matrix to solve linear equation to find optimal point of new vertex
            Eigen::Matrix4d A;
            A.row(0) << Q.row(0);
            A.row(1) << Q.row(1);
            A.row(2) << Q.row(2);
            A.row(3) << 0, 0, 0, 1;
            // Target : position of new vertex
            Eigen::Vector4d target = A.inverse() * Eigen::Vector4d(0, 0, 0, 1);
            // v^T Q v
            double cost = target.transpose() * Q * target;

            pq.push({v1, v2, target, cost});

            // Add the pair to the valid pairs
            validPairs[v1].insert(v2);
            validPairs[v2].insert(v1);
        }
    }

    int target_num_faces = 4;

    /*
     * edge collapsing
     */
    V_out = V;
    F_out = F;
    while (true) {
        // If terminal condition met, break
        if (F_out.rows() < target_num_faces) break;

        Pair pair = pq.top();
        pq.pop();

        if(pair.v1>pair.v2) {
            int tmp;
            tmp = pair.v2;
            pair.v2 = pair.v1;
            pair.v1 = tmp;
        }

        // If (v1,v2) is not valid pair, pass
        if(validPairs[pair.v1].count(pair.v2) == 0) continue;

        // 3D homogeneous coord to 3D coord and assign to v1 (we choose to remain v1 as target and delete v2)
        V_out.row(pair.v1) = pair.target.head<3>() / pair.target.w();

        // Update the faces in F_out
        for (int i = 0; i < F_out.rows(); i++) {
            for (int j = 0; j < 3; j++) {
                if (F_out(i, j) == pair.v2) {
                    // if Faces which has v2, modify it to index of v1 (target)
                    F_out(i, j) = pair.v1;
                }
            }
        }

        // Remove the pair from the valid pairs
        validPairs[pair.v1].erase(pair.v2);
        validPairs[pair.v2].erase(pair.v1);

        /*
         * cost update
         */

        // Update the quadric for v1 (Q' = Q1 + Q2)
        // 이전의 Q1 + Q2는 vTQv 를 계산해 pq에 넣을 때 적용됨, 선택당했으니 quadric에서 update
        quadrics[pair.v1] = quadrics[pair.v1] + quadrics[pair.v2];

        for (int v: validPairs[pair.v1]) {
            // Remove the old pair from the queue and the valid pairs
            validPairs[v].erase(pair.v1);

            // Calculate the new cost
            Eigen::Matrix4d Q = quadrics[pair.v1] + quadrics[v];

            // Matrix to solve linear equation to find optimal point of new vertex
            Eigen::Matrix4d A;
            A << Q.row(0),
                    Q.row(1),
                    Q.row(2),
                    Eigen::RowVector4d(0, 0, 0, 1);
            Eigen::Vector4d target = A.inverse() * Eigen::Vector4d(0, 0, 0, 1);

            // compute vTQv and push into pq
            double cost = target.transpose() * Q * target;

            // Add the new pair to the queue and the valid pairs
            pq.push({pair.v1, v, target, cost});
            validPairs[pair.v1].insert(v);
            validPairs[v].insert(pair.v1);
        }
    }

    // Remove degenerate faces from F_out
    for (int i = 0; i < F_out.rows(); ++i) {
        if (F_out(i, 0) == F_out(i, 1) || F_out(i, 0) == F_out(i, 2) || F_out(i, 1) == F_out(i, 2)) {
            F_out.row(i) = F_out.row(F_out.rows() - 1);
            F_out.conservativeResize(F_out.rows() - 1, Eigen::NoChange);
            i--;  // Check this row again
        }
    }

    igl::writeOBJ("../model/output/output.obj",V_out,F_out);

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V_out, F_out);
    viewer.data().set_face_based(true);
    viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
    viewer.launch();
}