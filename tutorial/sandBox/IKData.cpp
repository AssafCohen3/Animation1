#include "IKData.h"
#include <GLFW/glfw3.h>
#include "igl/opengl/MeshGL.h"

Eigen::Vector3d KeyToDirection(int key) {
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


IKData::IKData():
	id(-1)
	{}


void IKData::InitData(igl::opengl::ViewerData& vd, double linkLengthZ, double linkWidth) {
	id = vd.id;
	DrawAxis(vd, linkLengthZ, linkWidth);
}

void IKData::DrawAxis(igl::opengl::ViewerData& vd, double linkLengthZ, double linkWidth) {
	Eigen::MatrixXd axisPoints(6, 3);
	axisPoints.row(0) = Eigen::Vector3d(0, 0, -linkLengthZ);
	axisPoints.row(1) = Eigen::Vector3d(0, 0, linkLengthZ);
	axisPoints.row(2) = Eigen::Vector3d(linkWidth, 0, -linkLengthZ/2);
	axisPoints.row(3) = Eigen::Vector3d(-linkWidth, 0, -linkLengthZ / 2);
	axisPoints.row(4) = Eigen::Vector3d(0, linkWidth, -linkLengthZ / 2);
	axisPoints.row(5) = Eigen::Vector3d(0, -linkWidth, -linkLengthZ / 2);
	Eigen::MatrixXi axisEdges(3, 5);
	axisEdges <<
		0, 1, 1, 0, 0,
		2, 3, 0, 1, 0,
		4, 5, 0, 0, 1;
	vd.add_points(axisPoints, Eigen::RowVector3d(1, 0, 0));
	for (unsigned i = 0; i < axisEdges.rows(); i++) {
		vd.add_edges(axisPoints.row(axisEdges(i, 0)), axisPoints.row(axisEdges(i, 1)), Eigen::RowVector3d(axisEdges(i, 2), axisEdges(i, 3), axisEdges(i, 4)));
	}
}

void IKData::setTip(Eigen::Vector4d newTip) {
	tip = newTip;
}

Eigen::Vector4d IKData::getTip() {
	return tip;
}

Eigen::Matrix3d getPhiMatrix(double d) {
	Eigen::Matrix3d toRet;
	toRet.row(0) = Eigen::RowVector3d(1, 0, 0);
	toRet.row(1) = Eigen::RowVector3d(0, cos(d), -sin(d));
	toRet.row(2) = Eigen::RowVector3d(0, sin(d), cos(d));
	return toRet;
}

Eigen::Matrix3d getThetaMatrix(double d) {
	Eigen::Matrix3d toRet;
	toRet.col(0) = Eigen::Vector3d(cos(d), sin(d), 0);
	toRet.col(1) = Eigen::Vector3d(-sin(d), cos(d), 0);
	toRet.col(2) = Eigen::Vector3d(0, 0, 1);
	return toRet;
}

void IKData::printRotationMatrix(igl::opengl::ViewerData& vd) {
	Eigen::Vector3d ea = vd.GetRotation().eulerAngles(2, 0, 2);
	std::cout << "rotation matrices of " << id << ':' << std::endl;
	std::cout << vd.GetRotation() << std::endl;
	std::cout << "phi: " << std::endl;
	std::cout << getPhiMatrix(ea[0]) << std::endl;
	std::cout << "theta: " << std::endl;
	std::cout << getThetaMatrix(ea[1]) << std::endl;
}

void IKData::Animate(igl::opengl::glfw::Viewer& scn, igl::opengl::ViewerData& vd) {
}