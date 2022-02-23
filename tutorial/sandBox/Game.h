#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
#include "igl/readOBJ.h"
#include "igl/readTGF.h"
#include "igl/readDMAT.h"
#include "igl/PI.h"
#include "igl/lbs_matrix.h"
#include "igl/directed_edge_parents.h"
#include "igl/directed_edge_orientations.h"
#include "igl/deform_skeleton.h"
#include "Snake.h"
#include "PriceObject.h"

#define TUTORIAL false


class Game : public igl::opengl::glfw::Viewer
{
public:
	typedef igl::opengl::glfw::Viewer super;
	typedef
		std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
		RotationList;
	Game();
	~Game();
	virtual bool load_mesh_from_file(const std::string& mesh_file_name) override;
	void InitMesh();
	void Init(const std::string& config);
	void keyPressed(int key);
private:
	// Prepare array-based edge data structures and priority queue
	Snake* snake;
	std::vector<PriceObject*> prices;
	std::vector<std::vector<bool>> levelsConfig;
	int points;
	void Animate();


};