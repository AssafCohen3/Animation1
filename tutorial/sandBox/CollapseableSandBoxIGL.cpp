#include "tutorial/sandBox/CollapseableSandBoxIGL.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include <functional>
#include "igl/shortest_edge_and_midpoint.h"


CollapseableSandBoxIGL::CollapseableSandBoxIGL():
	collapsable_data_list(1)
{
}

void CollapseableSandBoxIGL::Init(const std::string& config)
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
		}
		nameFileout.close();
		collapsable_data_list.resize(data_list.size());
		for (int i = 0; i < data_list.size(); i++) {
			collapsable_data_list[i].InitData(data_list[i].V, data_list[i].F);
		}
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);

	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

}

CollapseableMeshDataIGL& CollapseableSandBoxIGL::collapseable_data() {
	return collapsable_data_list[selected_data_index];
}

void CollapseableSandBoxIGL::Simplify() {
	Simplify(std::ceil(0.05 * collapseable_data().Q.size()));
}

void CollapseableSandBoxIGL::Simplify(int faces_to_delete) {
	bool updated = false;
	Eigen::MatrixXd temp_V = data().V;
	Eigen::MatrixXi temp_F = data().F;
	for (int i = 0; i < faces_to_delete; i++) {
		if (!igl::collapse_edge(
			//TODO this maybe fail
			igl::shortest_edge_and_midpoint,
			temp_V, temp_F, collapseable_data().E,
			collapseable_data().EMAP, collapseable_data().EF,
			collapseable_data().EI, collapseable_data().Q,
			collapseable_data().Qit, collapseable_data().C)) {
			break;
		}
		updated = true;
	}
	if (updated) {
		data().clear();
		data().set_mesh(temp_V, temp_F);
		data().set_face_based(true);
		data().dirty = 157;
	}
}


void CollapseableSandBoxIGL::Animate()
{
	if (isActive)
	{



	}
}


