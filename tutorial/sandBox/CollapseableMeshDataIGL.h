#pragma once
#include <Eigen/core>
#include "set"
#include "vector"

class CollapseableMeshDataIGL{
public:
	void InitData(Eigen::MatrixXd _V, Eigen::MatrixXi _F);


    Eigen::MatrixXi E;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi EF;
    Eigen::MatrixXi EI;
    std::set<std::pair<double, int> > Q;
    std::vector<std::set<std::pair<double, int> >::iterator > Qit;
    Eigen::MatrixXd C;
};