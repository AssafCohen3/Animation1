#pragma once
#include <Eigen/core>
#include "set"
#include "vector"
#include "igl/opengl/ViewerData.h"
#include "igl/AABB.h"
#include "igl/opengl/glfw/Viewer.h"

class IKData {
public:
    IKData();
    void InitData(igl::opengl::ViewerData& vd, double linkLengthZ, double linkWidth);
    void Animate(igl::opengl::glfw::Viewer& scn, igl::opengl::ViewerData& vd);
    void DrawAxis(igl::opengl::ViewerData& vd, double linkLengthZ, double linkWidth);
    void printRotationMatrix(igl::opengl::ViewerData& vd);
    Eigen::Vector4d getTip();
    void setTip(Eigen::Vector4d newTip);
    int id;
    Eigen::Vector4d tip;
};