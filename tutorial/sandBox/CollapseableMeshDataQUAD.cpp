#include "CollapseableMeshDataQUAD.h"
#include <igl/edge_flaps.h>
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/vertex_triangle_adjacency.h"
#include "igl/collapse_edge.h"
#include <iostream>
#include "igl/circulation.h"

Eigen::Matrix4d ComputeKP(const Eigen::Vector3d plane, const Eigen::RowVector3d vertex) {
    double a = plane(0), b = plane(1), c = plane(2);
    double x = vertex(0), y = vertex(1), z = vertex(2);
    double d = -(a * x + b * y + c * z);
    Eigen::Matrix4d KP;
    KP << a * a, a * b, a * c, a * d,
          a * b, b * b, b * c, b * d,
          a * c, b * c, c * c, c * d,
          a * d, d * b, d * c, d * d;
    return KP;
}

Eigen::Matrix4d ComputeQ(const Eigen::RowVector3d vertex, const std::vector<int> planes, const Eigen::MatrixXd& _F_normals) {
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    for (int i : planes) {
        Q += ComputeKP(_F_normals.row(i).normalized(), vertex);
    }
    return Q;
}

double ComputeCost(const Eigen::RowVector3d v, const Eigen::Matrix4d Q) {
    Eigen::RowVector4d v_with_w;
    v_with_w << v, 1;
    return v_with_w * Q * v_with_w.transpose();
}

void CollapseableMeshDataQUAD::ComputeQs(const Eigen::MatrixXd& _V, const std::vector<std::vector<int>>& VF, const Eigen::MatrixXd &_F_noromals) {
    Q_Errors.resize(_V.rows());
    for (int i = 0; i < _V.rows(); i++) {
        Q_Errors[i] = ComputeQ(_V.row(i).transpose(), VF[i], _F_noromals);
    }
}

void CollapseableMeshDataQUAD::ComputeCostAndPlacement(const Eigen::MatrixXd& _V, const Eigen::Vector2i edge, double& cost, Eigen::RowVectorXd& p) {
    Eigen::Matrix4d v_Q = Q_Errors[edge(0)];
    Eigen::Matrix4d u_Q = Q_Errors[edge(1)];
    Eigen::Matrix4d contraction_Q = v_Q + u_Q;
    Eigen::Matrix4d Q_Equations = contraction_Q;
    Q_Equations.row(3) = Eigen::RowVector4d(0, 0, 0, 1);
    Eigen::Matrix4d contraction_Q_inverse;
    bool invertible;
    Q_Equations.computeInverseWithCheck(contraction_Q_inverse, invertible);
    if (invertible) {
        // succeded. optimal place = c_Q * [0 0 0 1]^T
        p = contraction_Q_inverse.block(0, 3, 3, 1).transpose();
        cost = ComputeCost(p, contraction_Q);
    }
    else {
        // select between endpoints and middlepoint
        Eigen::RowVector3d v, u, middlepoint;
        double vcost, ucost, middlepointcost;
        v = _V.row(edge(0)).transpose();
        u = _V.row(edge(1)).transpose();
        middlepoint = (u + v) / 2;
        vcost = ComputeCost(v, contraction_Q);
        ucost = ComputeCost(u, contraction_Q);
        middlepointcost = ComputeCost(middlepoint, contraction_Q);
        p = v;
        cost = vcost;
        if (ucost < cost) {
            p = u;
            cost = ucost;
        }
        if (middlepointcost < cost) {
            p = middlepoint;
            cost = middlepointcost;
        }
    }
}

void CollapseableMeshDataQUAD::InitData(Eigen::MatrixXd _V, Eigen::MatrixXi _F, const Eigen::MatrixXd _F_normals) {
    igl::edge_flaps(_F, E, EMAP, EF, EI);
    C.resize(E.rows(), _V.cols());
    Q.clear();
    Qit.resize(E.rows());
    std::vector<std::vector<int>> VF, VFi;
    igl::vertex_triangle_adjacency(_V, _F, VF, VFi);
    ComputeQs(_V, VF, _F_normals);

    for (int i = 0; i < E.rows(); i++) {
        double cost;
        Eigen::RowVectorXd new_place = Eigen::RowVector3d::Zero();
        ComputeCostAndPlacement(_V, E.row(i), cost, new_place);
        C.row(i) << new_place;
        Qit[i] = Q.insert(std::pair<double, int>(cost, i)).first;
    }
}

bool CollapseableMeshDataQUAD::collapse_edge(
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    Eigen::RowVector3d& new_vertex, int& collapsed_edge, double& cost){
    if (Q.empty() || (*(Q.begin())).first == std::numeric_limits<double>::infinity())
    {
        return false;
    }
    std::pair<double, int> p = *(Q.begin());
    int edge_idx = p.second;
    Q.erase(Q.begin());
    Qit[edge_idx] = Q.end();
    int v = E.row(edge_idx)[0];
    int u = E.row(edge_idx)[1];
    std::vector<int> N = igl::circulation(edge_idx, true, EMAP, EF, EI);
    std::vector<int> Nd = igl::circulation(edge_idx, false, EMAP, EF, EI);
    N.insert(N.begin(), Nd.begin(), Nd.end());
    int e1, e2, f1, f2;
    bool collapsed = false;
    new_vertex = C.row(edge_idx);
    collapsed_edge = edge_idx;
    cost = p.first;
    Eigen::RowVectorXd new_p = Eigen::Vector3d::Zero(3);
    if (igl::collapse_edge(edge_idx, C.row(edge_idx), V, F, E, EMAP, EF, EI, e1, e2, f1, f2)) {
        collapsed = true;
        Eigen::Matrix4d contraction_Q = Q_Errors[v] + Q_Errors[u];
        Q_Errors[v] = contraction_Q;
        Q_Errors[u] = contraction_Q;
        Q.erase(Qit[e1]);
        Qit[e1] = Q.end();
        Q.erase(Qit[e2]);
        Qit[e2] = Q.end();

        for (auto face : N) {
            if (F(face, 0) != IGL_COLLAPSE_EDGE_NULL ||
                F(face, 1) != IGL_COLLAPSE_EDGE_NULL ||
                F(face, 2) != IGL_COLLAPSE_EDGE_NULL)
            {
                for (int v = 0; v < 3; ++v) {
                    int ei = EMAP(v * F.rows() + face);
                    Q.erase(Qit[ei]);
                    ComputeCostAndPlacement(V, E.row(ei), cost, new_p);
                    Qit[ei] = Q.insert(std::pair<double, int>(cost, ei)).first;
                    C.row(ei) = new_p;
                }
            }
        }
    }
    else
    {
        p.first = std::numeric_limits<double>::infinity();
        Qit[edge_idx] = Q.insert(p).first;
    }
    return collapsed;
}

