#pragma once
#include <Eigen/core>
#include "set"
#include "vector"

class CollapseableMeshDataQUAD {
public:
    void InitData(const Eigen::MatrixXd _V, const Eigen::MatrixXi _F, const Eigen::MatrixXd _F_normals);
    void ComputeQs(const Eigen::MatrixXd& _V, const std::vector<std::vector<int>>& VF, const Eigen::MatrixXd& _F_noromals);
    void ComputeCostAndPlacement(const Eigen::MatrixXd& _V, const Eigen::Vector2i edge, double& cost, Eigen::RowVectorXd& p);
    bool collapse_edge(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::RowVector3d& new_vertex, int& collapsed_edge, double& cost);
    Eigen::MatrixXi E;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi EF;
    Eigen::MatrixXi EI;
    std::set<std::pair<double, int> > Q;
    std::vector<std::set<std::pair<double, int> >::iterator > Qit;
    Eigen::MatrixXd C;
    std::vector<Eigen::MatrixXd> Q_Errors;
};