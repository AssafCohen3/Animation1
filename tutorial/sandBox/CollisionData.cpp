#include "CollisionData.h"
#include <GLFW/glfw3.h>
#include <tutorial/sandBox/CollisionSandBox.h>
#include "igl/opengl/MeshGL.h"

#define BOX_EDGES    12
#define BOX_VERTICES 8

Eigen::Vector3d KeyToTranslation(int key) {
	switch (key)
	{
	case GLFW_KEY_UP:
		return Eigen::Vector3d(0, 1, 0);
	case GLFW_KEY_DOWN:
		return Eigen::Vector3d(0, -1, 0);
	case GLFW_KEY_LEFT:
		return Eigen::Vector3d(-1, 0, 0);
	case GLFW_KEY_RIGHT:
		return Eigen::Vector3d(1, 0, 0);
	case 'W':
	case 'w':
		return Eigen::Vector3d(0, 0, 1);
	case 'S':
	case 's':
		return Eigen::Vector3d(0, 0, -1);
	default:
		return Eigen::Vector3d(0, 0, 0);
	}
}


CollisionData::CollisionData():
	velocity(0.0),
	direction(0),
	id(-1),
	intersection_box_visible(false) {}


void CollisionData::InitData(igl::opengl::ViewerData& vh) {
	tree.init(vh.V, vh.F);
	id = vh.id;
	DrawBoundingBox(vh, tree.m_box, Eigen::RowVector3d(1, 0, 0));
}

void CollisionData::DrawBoundingBox(igl::opengl::ViewerData& vd, Eigen::AlignedBox<double, 3> to_draw, Eigen::RowVector3d color) {
	Eigen::MatrixXd V_box(BOX_VERTICES, 3);
	for (int i = 0; i < BOX_VERTICES; i++) {
		V_box.row(i) = to_draw.corner(static_cast<Eigen::AlignedBox3d::CornerType>(i));
	}
	// Edges of the bounding box
	Eigen::MatrixXi E_box(BOX_EDGES, 2);
	E_box <<
		0, 1,
		1, 3,
		2, 3,
		2, 0,
		4, 5,
		5, 7,
		6, 7,
		6, 4,
		2, 6,
		0, 4,
		1, 5,
		7, 3;
	vd.add_points(V_box, color);
	for (unsigned i = 0; i < E_box.rows(); i++) {
		vd.add_edges(V_box.row(E_box(i, 0)), V_box.row(E_box(i, 1)), color);
	}
}

void CollisionData::EraseIntersectionBox(igl::opengl::ViewerData& vd) {
	if (intersection_box_visible) {
		vd.points.conservativeResize(vd.points.rows() - BOX_VERTICES, 6);
		vd.dirty |= igl::opengl::MeshGL::DIRTY_OVERLAY_POINTS;
		vd.lines.conservativeResize(vd.lines.rows() - BOX_EDGES, 9);
		vd.dirty |= igl::opengl::MeshGL::DIRTY_OVERLAY_LINES;
		intersection_box_visible = false;
	}
}

void CollisionData::Animate(igl::opengl::glfw::Viewer& scn, igl::opengl::ViewerData& vd) {
	if (velocity > 0) {
		if (intersection_box_visible) {
			EraseIntersectionBox(vd);
		}
		vd.TranslateInSystem(movement_rotation, KeyToTranslation(direction) * velocity, true);
		scn.WhenTranslate(vd.id);
	}
}

void CollisionData::SetVelocity(double new_velocity) { velocity = new_velocity; }
void CollisionData::SetDirection(int new_direction, Eigen::Matrix3d rotation_mat) { 
	// accelerate if the same direction
	if (direction == new_direction && velocity > 0) {
		velocity *= 2;
	}
	else {
		velocity = INIT_VELOCITY;
	}
	direction = new_direction; 
	// save the rotation at the moment of the press to keep in straight line
	movement_rotation = rotation_mat;
}
bool CollisionData::IsIntersectionBoxVisible() {
	return intersection_box_visible;
}
