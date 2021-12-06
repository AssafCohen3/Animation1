#pragma once
#include <tutorial/sandBox/CollapseableMeshDataQUAD.h>
#include <tutorial/sandBox/CollapseableSandBox.h>

class CollapseableSandBoxQUAD : public CollapseableSandBox
{
public:
	CollapseableSandBoxQUAD();
	void Init(const std::string& config);
	void Simplify();
	void Simplify(int faces_to_delete);
	CollapseableMeshDataQUAD& collapseable_data();

	std::vector<CollapseableMeshDataQUAD> collapsable_data_list;
	double doubleVariable;
private:
	// Prepare array-based edge data structures and priority queue


	void Animate();
};