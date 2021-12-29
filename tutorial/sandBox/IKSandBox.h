#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
#include <tutorial/sandBox/IKData.h>

class IKSandBox : public igl::opengl::glfw::Viewer
{
public:
	typedef igl::opengl::glfw::Viewer super;
	IKSandBox(int links_to_initiate);
	~IKSandBox();
	virtual bool load_mesh_from_file(const std::string& mesh_file_name) override;
	virtual void open_dialog_load_mesh() override;
	virtual void WhenTranslate(int mesh_id) override;
	IKData* ik_data();
	void InitMesh();
	void InitIKData(int dataid);
	void Init(const std::string& config);
	void updateTips(Eigen::MatrixXd baseTransformations, int firstIndex);
	bool checkIfTooFar();
	bool checkIfTargetReached();
	void CCD();
	void Fabrik();
	void UpdateDest();
	void printSceneRotation();
	void fixZAxisRotation();
	void printSceneTips();
	void printSceneDestination();
	double doubleVariable;
private:
	// Prepare array-based edge data structures and priority queue

	std::map<int, IKData*> links_data_map;
	double linksLengthAlongZ;
	double linkWidth;
	Eigen::Vector4d destinationPos;
	Eigen::Vector4d endTipPos;
	Eigen::Vector4d startTipPos;
	int links_to_initiate;
	int first_link;
	int last_link;
	int destinationMeshId;
	std::string system_center_mesh_file;
	std::string system_link_mesh_file;
	void Animate();


};