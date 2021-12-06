#pragma once
#include <tutorial/sandBox/CollapseableMeshDataIGL.h>
#include <tutorial/sandBox/CollapseableSandBox.h>

class CollapseableSandBoxIGL : public CollapseableSandBox
{
public:
	CollapseableSandBoxIGL();
	void Init(const std::string& config);
	void Simplify();
	void Simplify(int faces_to_delete);
	CollapseableMeshDataIGL& collapseable_data();

	std::vector<CollapseableMeshDataIGL> collapsable_data_list;
	double doubleVariable;
private:
	// Prepare array-based edge data structures and priority queue


	void Animate();
};