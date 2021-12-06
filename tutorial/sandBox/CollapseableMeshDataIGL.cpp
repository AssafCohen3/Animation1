#include "CollapseableMeshDataIGL.h"
#include <igl/edge_flaps.h>
#include "igl/shortest_edge_and_midpoint.h"

void CollapseableMeshDataIGL::InitData(Eigen::MatrixXd _V, Eigen::MatrixXi _F) {
	igl::edge_flaps(_F, E, EMAP, EF, EI);
	C.resize(E.rows(), _V.cols());
	Q.clear();
    Qit.resize(E.rows());
    for (int i = 0; i < E.rows(); i++) {
        double cost = i;
        Eigen::RowVectorXd p(1, 3);
        igl::shortest_edge_and_midpoint(i, _V, _F, E, EMAP, EF, EI, cost, p);
        C.row(i) = p;
        Qit[i] = Q.insert(std::pair<double, int>(cost, i)).first;
    }
}