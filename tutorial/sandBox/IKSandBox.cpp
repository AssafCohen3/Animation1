#include "tutorial/sandBox/IKSandBox.h"
#include "Eigen/dense"
#include <functional>
#include <GLFW/glfw3.h>
#include "stack"

#define STEPS_MULTIPLIER	0.1
#define DELTA				0.1

IKSandBox::IKSandBox(int links_to_initiate):
	links_to_initiate(links_to_initiate),
	destinationMeshId(-1),
	first_link(-1),
	last_link(-1),
	linkWidth(-1),
	linksLengthAlongZ(-1)
{
#ifndef IGL_VIEWER_VIEWER_QUIET
	const std::string usage(R"(tutorial::sandBox::IKSandBox usage:
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

IKData* IKSandBox::ik_data() {
	if (data().id == destinationMeshId) {
		return NULL;
	}
	return links_data_map[data().id];
}

void IKSandBox::open_dialog_load_mesh() {
	int saved_idx = selected_data_index;
	load_mesh_from_file(system_link_mesh_file);
	parents[data().id] = last_link;
	last_link = data().id;
	data().show_faces ^= 2;
	data().show_texture ^= 2;
	data().show_overlay ^= 2;
	data().show_overlay_depth ^= 2;
	updateTips(Eigen::Matrix4d::Identity(), first_link);
	selected_data_index = saved_idx;
}

bool IKSandBox::load_mesh_from_file(const std::string& mesh_file_name) {
	if (super::load_mesh_from_file(mesh_file_name)) {
		parents.push_back(-1);
		data_list.back().set_visible(true, 2);
		InitMesh();
		InitIKData(data().id);
		return true;
	}
	return false;
}

void IKSandBox::Init(const std::string& config)
{
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file " << config << std::endl;
	}
	else
	{
		nameFileout >> system_center_mesh_file;
		nameFileout >> system_link_mesh_file;
		nameFileout.close();
		std::cout << "openning " << system_center_mesh_file << std::endl;
		load_mesh_from_file(system_center_mesh_file);
		for (int i = 1; i <= links_to_initiate; i++) {
			std::cout << "openning " << system_link_mesh_file << std::endl;
			load_mesh_from_file(system_link_mesh_file);
			parents[data().id] = last_link;
			last_link = data().id;
		}
	}
	updateTips(Eigen::Matrix4d::Identity(), first_link);
	MyTranslate(Eigen::Vector3d(0, 0, -10), true);
	selected_data_index = 0;
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	isActive = false;
}

void IKSandBox::InitMesh(){
	data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
	data().point_size = 10;
	data().line_width = 2;
	data().set_visible(false, 1);
}
void IKSandBox::InitIKData(int data_id) {
	if (data_id == 0) {
		destinationMeshId = data_id;
		data(data_id).TranslateInSystem(GetRotation(), Eigen::Vector3d(5, 0, 0), true);
		UpdateDest();
	}
	else{
		if (first_link == -1) {
			Eigen::Vector3d minVals = data().V.colwise().minCoeff();
			Eigen::Vector3d maxVals = data().V.colwise().maxCoeff();
			Eigen::Vector3d meanVals = data().V.colwise().mean();
			linksLengthAlongZ = maxVals(2) - minVals(2);
			linkWidth = maxVals(0) - minVals(0);
			endTipPos = Eigen::Vector4d(meanVals(0), meanVals(1), minVals(2), 1);
			startTipPos = Eigen::Vector4d(meanVals(0), meanVals(1), maxVals(2), 1);
			first_link = data_id;
		}
		IKData* new_data = new IKData();
		new_data->InitData(data(data_id), linksLengthAlongZ, linkWidth);
		links_data_map[data_id] = new_data;
		
		data(data_id).TranslateInSystem(data().GetRotation(), Eigen::RowVector3d(0, 0, -linksLengthAlongZ), true);
		data(data_id).SetCenterOfRotation(Eigen::RowVector3d(0, 0, linksLengthAlongZ/2)); // each link rotate around the joint with the parent link
	}
}

void IKSandBox::UpdateDest() {
	destinationPos = data(destinationMeshId).MakeTransd().col(3);
}

void IKSandBox::WhenTranslate(int mesh_id) {
	if(mesh_id == destinationMeshId){
		UpdateDest();
	}
	else {
		updateTips(CalcParentsTrans(mesh_id), mesh_id);
	}
}

void IKSandBox::updateTips(Eigen::MatrixXd baseTransformations, int firstIndex) {
	for (int i = firstIndex; i <= last_link; i++) {
		baseTransformations *= data(i).MakeTransd();
		links_data_map[i]->setTip(baseTransformations * endTipPos);
	}
}

bool IKSandBox::checkIfTargetReached() {
	Eigen::Vector4d finalDistanceV = destinationPos - links_data_map[last_link]->getTip();
	double distance = finalDistanceV.norm();
	if (distance <= DELTA) {
		std::cout << "reached target. distance: " << distance << std::endl;
		isActive = false;
		return true;
	}
	return false;
}

bool IKSandBox::checkIfTooFar() {
	Eigen::Vector4d basePos = data(first_link).MakeTransd() * startTipPos;
	Eigen::Vector4d Vtbase = destinationPos - basePos;
	double currentDestination = Vtbase.norm();
	if (currentDestination > (last_link - first_link + 1) * linksLengthAlongZ) {
		std::cout << "too far from destination. try add more links. current base distance from target: " << currentDestination << std::endl;
		isActive = false;
		return true;
	}
	return false;
}

void IKSandBox::CCD() {
	if (checkIfTargetReached()) {
		fixZAxisRotation();
		updateTips(Eigen::Matrix4d::Identity(), first_link);
		return;
	}
	else if (checkIfTooFar()) {
		return;
	}
	Eigen::Vector4d basePos = data(first_link).MakeTransd() * startTipPos;
	for (int i = last_link-1; i >= first_link - 1; i--) {
		Eigen::Vector4d currentTip = i == first_link - 1 ? basePos : links_data_map[i]->getTip();
		Eigen::Vector4d Ve, Vt, Vr; 
		Ve << (links_data_map[last_link]->getTip() - currentTip);
		Vt << (destinationPos - currentTip);
		Vr << Ve.cross3(Vt).normalized();
		Eigen::MatrixXd currentLinkRotation = CalcParentsTrans(i + 1);
		Eigen::MatrixXd nextLinkRotation = currentLinkRotation * data(i + 1).MakeTransd();
		Eigen::Vector4d eulerAxis = nextLinkRotation.inverse() * Vr;
		double dotProduct = Ve.normalized().dot(Vt.normalized());
		if (abs(dotProduct) > 1) {
			dotProduct = dotProduct < 0 ? -1 : 1;
		}
		double angleToRotate = acos(dotProduct);
		data(i + 1).MyRotate(eulerAxis.head(3), angleToRotate * STEPS_MULTIPLIER);
		updateTips(currentLinkRotation, i + 1);
	}
}


void IKSandBox::Fabrik() {
	if (checkIfTargetReached()) {
		fixZAxisRotation();
		updateTips(Eigen::Matrix4d::Identity(), first_link);
		return;
	}
	else if (checkIfTooFar()) {
		return;
	}
	Eigen::Vector4d basePos = data(first_link).MakeTransd() * startTipPos;
	Eigen::Vector4d b = basePos;
	std::map<int, Eigen::Vector4d> new_points;
	new_points[last_link] = destinationPos;
	for (int i = last_link - 1; i >= first_link - 1; i--) {
		Eigen::Vector4d currentTip = i == first_link - 1 ? basePos : links_data_map[i]->getTip();
		Eigen::Vector4d nextTip = new_points[i + 1];
		Eigen::Vector4d Vri = nextTip - currentTip;
		double lambdai = linksLengthAlongZ / Vri.norm();
		new_points[i] = (1 - lambdai) * nextTip + lambdai * currentTip;
	}
	new_points[first_link - 1] = b;
	for (int i = first_link-1; i <= last_link - 1; i++) {
		Eigen::Vector4d currentTip = new_points[i];
		Eigen::Vector4d nextTip = new_points[i + 1];
		Eigen::Vector4d Vri = nextTip - currentTip;
		double lambdai = linksLengthAlongZ / Vri.norm();
		new_points[i+1] = (1 - lambdai) * currentTip + lambdai * nextTip;
	}
	for(int i = first_link-1; i <= last_link - 1; i++)
	{
		Eigen::Vector4d Ve = links_data_map[i+1]->getTip() - (i == first_link - 1 ? basePos : links_data_map[i]->getTip());
		Eigen::Vector4d Vt = new_points[i + 1] - new_points[i];
		Eigen::Vector4d Vr;
		Vr << Ve.cross3(Vt).normalized();
		Eigen::MatrixXd currentLinkRotation = CalcParentsTrans(i + 1);
		Eigen::MatrixXd nextLinkRotation = currentLinkRotation * data(i + 1).MakeTransd();
		Eigen::Vector4d eulerAxis = nextLinkRotation.inverse() * Vr;
		double dotProduct = Ve.normalized().dot(Vt.normalized());
		if (abs(dotProduct) > 1) {
			dotProduct = dotProduct < 0 ? -1 : 1;
		}
		double angleToRotate = acos(dotProduct);
		data(i + 1).MyRotate(eulerAxis.head(3), angleToRotate * STEPS_MULTIPLIER);
		updateTips(currentLinkRotation, i + 1);
	}
}

void IKSandBox::fixZAxisRotation() {
	for (int i = first_link; i <= last_link; i++)
	{
		Eigen::Vector3d ea = data(i).GetRotation().eulerAngles(2, 0, 2);
		data(i).MyRotate(Eigen::Vector3d(0, 0, 1), -ea[2]);
		if (i < last_link) {
			data(i+1).RotateInSystem(Eigen::Vector3d(0, 0, 1), ea[2]);
		}
	}
}

void IKSandBox::printSceneRotation() {
	if (selected_data_index < first_link) {
		std::cout << "whole scene rotation: " << std::endl;
		std::cout << GetRotation() << std::endl;
	}
	else {
		links_data_map[selected_data_index]->printRotationMatrix(data());
	}
}

void IKSandBox::printSceneDestination() {
	std::cout << "scene destination: " << std::endl;
	std::cout << destinationPos << std::endl;
}

void IKSandBox::printSceneTips() {
	for (int i = first_link; i <= last_link; i++) {
		std::cout << "link " << i << " tips:" << std::endl;
		std::cout << links_data_map[i]->getTip() << std::endl;
	}
}

IKSandBox::~IKSandBox()
{
	for (std::pair<int, IKData*> col_data : links_data_map) {
		delete col_data.second;
	}
}

void IKSandBox::Animate()
{
	if (isActive)
	{
		Fabrik();
	}
}


