#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"

class ExtendedSandBox : public igl::opengl::glfw::Viewer
{
public:
	ExtendedSandBox();
	~ExtendedSandBox();
	void Init(const std::string& config);
	double doubleVariable;
private:
	// Prepare array-based edge data structures and priority queue


	void Animate();


};