#pragma once
#include <Eigen/core>
#include "set"
#include "vector"
#include "igl/opengl/ViewerData.h"
#include "igl/AABB.h"
#include "igl/opengl/glfw/Viewer.h"

class CollisionData {
public:
    CollisionData();
    void InitData(igl::opengl::ViewerData& vd);
    void DrawBoundingBox(igl::opengl::ViewerData& vd, Eigen::AlignedBox<double, 3> to_draw, Eigen::RowVector3d color);
    void EraseIntersectionBox(igl::opengl::ViewerData& vd);
    void Animate(igl::opengl::glfw::Viewer& scn, igl::opengl::ViewerData& vd);
    void SetDirection(int new_direction, Eigen::Matrix3d rotation_mat);
    void SetVelocity(double new_velocity);
    bool IsIntersectionBoxVisible();
    igl::AABB<Eigen::MatrixXd, 3> tree;
    int id;
    int direction;
    double velocity;
    bool intersection_box_visible;
    Eigen::Matrix3d movement_rotation;
};