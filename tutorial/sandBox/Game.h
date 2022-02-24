#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include <igl/opengl/ViewerCore.h>
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

#define PRICES_NUMBER      10
#define MAX_LEVEL          8
#define X_RANGE            40
#define Y_RANGE            20
#define Z_RANGE            40
#define MAX_TRAVEL_DIST    10
#define MIN_TRAVEL_DIST    5
#define MIN_PRICE_VELOCITY 0.1
#define MAX_PRICE_VELOCITY 0.3
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
	void InitializeGame();
	void keyPressed(int key);
	void StartLevel();
	double GenerateXCord();
	double GenerateYCord();
	double GenerateZCord();
	double GenerateVelocity();
	Eigen::Vector3d GenerateDirection();
	double GenerateTravelDistance();
	int GetCurrentLevel();
	int GetGamePoints();
	int GetLevelPoints();
	bool HasMoreLevels();
	void DetectCollisions();
	bool FindCollision(int priceId);
	bool CheckIntersection(Eigen::AlignedBox<double, 3> snakeHeadBox, Eigen::AlignedBox<double, 3> priceBox, int priceId);
	bool IsLevelEnded();
	void SetVisibilities();
	void DrawSnakeToClosestPrice();
	void SetCameraView(igl::opengl::ViewerCore& core);
	bool UpdateCamera();
	void ToggleCamera();

private:
	Snake* snake;
	std::vector<PriceObject*> prices;
	int gamePoints;
	int levelPoints;
	int currentLevel;
	bool playing;
	void Animate();
	const Eigen::Vector3d priceDirections[4] = {Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, -1, 0)};
	long seed;
	bool firstPersonMode;

};