#include "tutorial/sandBox/CollapseableSandBoxQUAD.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include <functional>
#include "igl/shortest_edge_and_midpoint.h"


CollapseableSandBoxQUAD::CollapseableSandBoxQUAD() :
	collapsable_data_list(1)
{
}

void CollapseableSandBoxQUAD::Init(const std::string& config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file " << config << std::endl;
	}
	else
	{

		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(false, 1);
			std::cout << "vertices: " << data().V.rows() << ", traingles: " << data().F.rows() << std::endl;
		}
		nameFileout.close();
		collapsable_data_list.resize(data_list.size());
		for (int i = 0; i < data_list.size(); i++) {
			collapsable_data_list[i].InitData(data_list[i].V, data_list[i].F, data_list[i].F_normals);
		}
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);

	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

}

CollapseableMeshDataQUAD& CollapseableSandBoxQUAD::collapseable_data() {
	return collapsable_data_list[selected_data_index];
}

void CollapseableSandBoxQUAD::Simplify() {
	Simplify(std::ceil(0.05 * collapseable_data().Q.size()));
}

void CollapseableSandBoxQUAD::Simplify(int faces_to_delete) {
	bool updated = false;
	Eigen::MatrixXd temp_V = data().V;
	Eigen::MatrixXi temp_F = data().F;
	for (int i = 0; i < faces_to_delete; i++) {
		Eigen::RowVector3d p = Eigen::Vector3d::Zero();
		int e = 0;
		double cost = 0;
		if (!collapseable_data().collapse_edge(temp_V, temp_F, p, e, cost)) {
			break;
		}
//		std::cout << "edge " << e << ", cost = " << cost << ", new v position (" << Eigen::RowVectorXd(p) << ")" << std::endl;
		updated = true;
	}
	if (updated) {
		data().clear();
		data().set_mesh(temp_V, temp_F);
		data().set_face_based(true);
		data().dirty = 157;
	}
}


void CollapseableSandBoxQUAD::Animate()
{
	if (isActive)
	{



	}
}


