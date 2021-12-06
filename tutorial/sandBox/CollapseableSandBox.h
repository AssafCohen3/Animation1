#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"

class CollapseableSandBox : public igl::opengl::glfw::Viewer
{
public:
	virtual void Simplify() = 0;
	virtual void Simplify(int faces_to_delete) = 0;
};