#include "tutorial/sandBox/CollisionSandBox.h"
#include "Eigen/dense"
#include <functional>
#include <GLFW/glfw3.h>
#include "stack"

CollisionSandBox::CollisionSandBox(){
#ifndef IGL_VIEWER_VIEWER_QUIET
	const std::string usage(R"(tutorial::sandBox::CollisionSandBox usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  c       Erase selected mesh intersection box
  C       Erase all intersection boxes
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  M,m     Toggle movement in the scene
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  ;       Toggle vertex labels
  :       Toggle face labels
  LEFT    Move selected mesh left
  RIGHT   Move selected mesh right
  UP      Move selected mesh up
  DOWN    Move selected mesh down
  W/w     Move selected mesh outward
  S/s     Move selected mesh inward
  [ press same direction for increasing velocity ]
  Q/q     Zero selected mesh velocity
  V/v     Reset selected mesh velocity)");
	std::cout << usage << std::endl;
#endif

}

CollisionData* CollisionSandBox::collision_data() {
	return collision_data_map[data().id];
}

bool CollisionSandBox::load_mesh_from_file(const std::string& mesh_file_name) {
	if (super::load_mesh_from_file(mesh_file_name)) {
		InitMesh();
		InitCollisionData(data().id);
		return true;
	}
	return false;
}

void CollisionSandBox::Init(const std::string& config)
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
			std::cout << "id: " << data().id << std::endl;
			parents.push_back(-1);
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -2), true);
	selected_data_index = 0;
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	isActive = true;
}

void CollisionSandBox::InitMesh(){
	data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
	data().point_size = 10;
	data().line_width = 2;
	data().set_visible(false, 1);
}
void CollisionSandBox::InitCollisionData(int data_id) {
	CollisionData* new_data = new CollisionData();
	double dist_from_center = (std::floor(data_id / 2.0) + 1) * (data_id % 2 == 0 ? -1 : 1);
	new_data->SetDirection(GLFW_KEY_RIGHT, GetRotation());
	new_data->SetVelocity(data_id == 0 ? INIT_VELOCITY : 0);
	new_data->InitData(data(data_id));
	data().SetCenterOfRotation(new_data->tree.m_box.center());
	data().TranslateInSystem(GetRotation(), Eigen::Vector3d(dist_from_center, 0, 0), true);
	collision_data_map[data_id] = new_data;
}

void CollisionSandBox::WhenTranslate(int mesh_id) {
	CollisionData* moved_mesh = collision_data_map[mesh_id];
	bool colided = false;
	for (int i = 0; i < data_list.size(); i++) {
		if(data_list[i].id == mesh_id) {
			continue;
		}
		if (FindCollision(moved_mesh, collision_data_map[data_list[i].id])) {
			collision_data_map[data_list[i].id]->SetVelocity(0);
			moved_mesh->SetVelocity(0);
		}
	}
}

bool CollisionSandBox::FindCollision(CollisionData* m1, CollisionData* m2) {
	typedef igl::AABB<Eigen::MatrixXd, 3>& AABB_TREE;
	// the stack will hold pairs of trees to check. we use stack for depth search
	std::stack<std::pair<AABB_TREE, AABB_TREE>> boxes_stack;
	boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(m1->tree, m2->tree));
	while (!boxes_stack.empty()) {
		std::pair<AABB_TREE, AABB_TREE> candidate_boxes = boxes_stack.top();
		boxes_stack.pop();
		if (CheckIntersection(candidate_boxes.first.m_box, candidate_boxes.second.m_box, m1->id, m2->id)) {
			if (candidate_boxes.first.is_leaf() && candidate_boxes.second.is_leaf()) {
				m1->DrawBoundingBox(data(m1->id), candidate_boxes.first.m_box, Eigen::RowVector3d(0, 1, 0));
				m2->DrawBoundingBox(data(m2->id), candidate_boxes.second.m_box, Eigen::RowVector3d(0, 1, 0));
				m1->intersection_box_visible = true;
				m2->intersection_box_visible = true;
				return true;
			}
			else if (candidate_boxes.first.is_leaf()) {
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(candidate_boxes.first, *candidate_boxes.second.m_right));
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(candidate_boxes.first, *candidate_boxes.second.m_left));
			}
			else if (candidate_boxes.second.is_leaf()) {
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_right, candidate_boxes.second));
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_left, candidate_boxes.second));
			}
			else {
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_right, *candidate_boxes.second.m_right));
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_right, *candidate_boxes.second.m_left));
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_left, *candidate_boxes.second.m_right));
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_left, *candidate_boxes.second.m_left));
			}
		}
	}
	return false;
}

bool CollisionSandBox::CheckIntersection(Eigen::AlignedBox<double, 3> A, Eigen::AlignedBox<double, 3> B, int a_id, int b_id) {
	// compute boxes center with respect to scales and translations
	Eigen::Vector3d A_initial_center = A.center();
	Eigen::Vector3d B_initial_center = B.center();
	Eigen::Vector4d A_expanded_center, B_expanded_center;
	A_expanded_center << A_initial_center, 1;
	B_expanded_center << B_initial_center, 1;
	Eigen::Vector3d a_center = (data(a_id).MakeTransScaled() * A_expanded_center).block(0,0,3,1);
	Eigen::Vector3d b_center = (data(b_id).MakeTransScaled() * B_expanded_center).block(0,0,3,1);
	// compute boxes dimensions with respect to scales
	Eigen::Vector3d sizes_a = data(a_id).MakeScaled() * A.sizes();
	Eigen::Vector3d sizes_b = data(b_id).MakeScaled() * B.sizes();
	// ccompute boxes axes with respect to rotation
	Eigen::Matrix3d units_a = data(a_id).GetRotation(); // unit vectors of A axes.
	Eigen::Matrix3d units_b = data(b_id).GetRotation(); // unit vectors of B axes.
	Eigen::Matrix3d r_matrix = units_a.transpose() * units_b; // axes dot products matrix
	Eigen::Vector3d T = b_center - a_center;

	// check for A parallel planes
	for (int i = 0; i < 3; i++) {
		double left_side = std::abs(T.dot(units_a.row(i)));
		double right_side = sizes_a(i) + std::abs(sizes_b(0) * r_matrix(i, 0)) + std::abs(sizes_b(1) * r_matrix(i, 1)) + std::abs(sizes_b(2) * r_matrix(i, 2));
		if (left_side > right_side)
			return false;
	}
	// check for B parallel planes
	for (int i = 0; i < 3; i++) {
		double left_side = std::abs(T.dot(units_b.row(i)));
		double right_side = sizes_b(i) + std::abs(sizes_a(0) * r_matrix(0, i)) + std::abs(sizes_a(1) * r_matrix(1, i)) + std::abs(sizes_a(2) * r_matrix(2, i));
		if (left_side > right_side)
			return false;
	}
	// check for edge pairs planes
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			double left_side = std::abs(
				T.dot(units_a.row((i + 2) % 3)) * r_matrix((i + 1) % 3, j) - 
				(T.dot(units_a.row((i + 1) % 3)) * r_matrix((i + 2) % 3, j)));

			double right_side =
				std::abs(sizes_a((i + 1) % 3) * r_matrix((i + 2) % 3, j)) +
				std::abs(sizes_a((i + 2) % 3) * r_matrix((i + 1) % 3, j)) +
				std::abs(sizes_b((j + 1) % 3) * r_matrix(i, (j + 2) % 3)) +
				std::abs(sizes_b((j + 2) % 3) * r_matrix(i, (j + 1) % 3));
			if (left_side > right_side) {
				return false;
			}
		}
	}
	return true;
}

void CollisionSandBox::EraseAllIntersectionBoxes() {
	for (auto pair : collision_data_map) {
		if (pair.second->IsIntersectionBoxVisible()) {
			pair.second->EraseIntersectionBox(data(pair.first));
		}
	}
}

void CollisionSandBox::ChangeDirection(int new_direction) {
	collision_data()->SetDirection(new_direction, GetRotation());
}

void CollisionSandBox::ModifyVelocity(int key) {
	collision_data()->SetVelocity(key == 'q' || key == 'Q' ? 0 : INIT_VELOCITY);
}

CollisionSandBox::~CollisionSandBox()
{
	for (std::pair<int, CollisionData*> col_data : collision_data_map) {
		delete col_data.second;
	}
}

void CollisionSandBox::Animate()
{
	if (isActive)
	{
		for (std::pair<int, CollisionData*> col_data : collision_data_map) {
			col_data.second->Animate(*this, data(col_data.first));
		}
	}
}


