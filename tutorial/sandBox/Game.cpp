#include "tutorial/sandBox/Game.h"
#include "Eigen/dense"
#include <functional>
#include <GLFW/glfw3.h>
#include "stack"

Game::Game() :
	snake(NULL),
	currentLevel(1),
	playing(false),
	gamePoints(0),
	levelPoints(0),
	seed(-1),
	firstPersonMode(false)
{
#ifndef IGL_VIEWER_VIEWER_QUIET
	//TODO update usage
	const std::string usage(R"(Controls:
  [SPACE] Stop/Continue
  C/c     Toggle between cameras
  LEFT    Move the Snake left
  RIGHT   Move the Snake right
  UP      Move the Snake up
  DOWN    Move the Snake down
  W/w     increase Snake speed
  S/s     decrease Snake speed)");
	std::cout << usage << std::endl;
#endif

}

Game::~Game()
{
	delete snake;
	for (int i = 0; i < PRICES_NUMBER; i++) {
		delete prices[i];
	}
}

bool Game::load_mesh_from_file(const std::string& mesh_file_name) {
	if (super::load_mesh_from_file(mesh_file_name)) {
		parents.push_back(-1);
		//data_list.back().set_visible(true, 2);
		InitMesh();
		return true;
	}
	return false;
}

void Game::Init(const std::string& config)
{
	std::ifstream nameFileout;
	std::string snakeMeshFile;
	std::string snakeTextureFile;
	std::string priceObjectMeshFile;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file " << config << std::endl;
	}
	else
	{
		nameFileout >> snakeMeshFile;
		nameFileout >> snakeTextureFile;
		nameFileout >> priceObjectMeshFile;
		nameFileout.close();
		std::cout << "openning " << snakeMeshFile << std::endl;
		load_mesh_from_file(snakeMeshFile);
		data().image_texture(snakeTextureFile);
		snake = new Snake(selected_data_index);
		data_list.back().set_visible(true, 2);
		prices.resize(PRICES_NUMBER);
		for (int i = 0; i < PRICES_NUMBER; i++) {
			load_mesh_from_file(priceObjectMeshFile);
			data().face_based = false;
			data().show_texture &=  ~0;
			data().set_visible(false, 2);
			prices[i] = new PriceObject(selected_data_index);
		}
	}
	InitializeGame();
	//MyTranslate(Eigen::Vector3d(0, 0, -10), true);
	selected_data_index = 0;
	//data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	isActive = true;
}

void Game::InitMesh() {
	data().point_size = 10;
	data().line_width = 2;
	data().show_overlay ^= 2;
	data().show_overlay_depth ^= 2;
	data().set_visible(false, 1);
}

void Game::InitializeGame() {
	snake->InitData(data(snake->GetId()));
	for (int i = 0; i < PRICES_NUMBER; i++) {
		prices[i]->Initialize(data(prices[i]->GetId()));
	}
	seed = time(NULL);
	srand(seed);
	currentLevel = 1;
	gamePoints = 0;
	levelPoints = 0;
	playing = false;
	std::cout << "Game Seed: " << seed << std::endl;
}

void Game::keyPressed(int key)
{
	switch (key)
	{
	case GLFW_KEY_UP:
	case GLFW_KEY_DOWN:
	case GLFW_KEY_LEFT:
	case GLFW_KEY_RIGHT:
		snake->updateDirection(data(snake->GetId()), key);
		break;
	case 'w':
	case 's':
		snake->updateVelocity(key);
		break;
	default:
		break;
	}
}

double Game::GenerateXCord() {
	return (rand() % (X_RANGE + 1)) - X_RANGE / 2.0;
}

double Game::GenerateYCord() {
	return (rand() % (Y_RANGE + 1)) - Y_RANGE / 2.0;
}

double Game::GenerateZCord() {
	return (rand() % (Z_RANGE + 1)) - Z_RANGE / 2.0;
}

double Game::GenerateVelocity() {
	return MIN_PRICE_VELOCITY + (((float)rand()) / ((float)RAND_MAX)) * (MAX_PRICE_VELOCITY - MIN_PRICE_VELOCITY);
}

double Game::GenerateTravelDistance() {
	return MIN_TRAVEL_DIST + (((float)rand()) / ((float)RAND_MAX)) * (MAX_TRAVEL_DIST - MIN_TRAVEL_DIST);
}

Eigen::Vector3d Game::GenerateDirection() {
	return priceDirections[rand() % 4];
}

void Game::SetCameraView(igl::opengl::ViewerCore& core){
	if (!firstPersonMode) {
		core.camera_eye = Eigen::Vector3f(0, 7, 1);
		core.camera_view_angle = 270;
		core.camera_center = Eigen::Vector3f(0, 0, 0);
		core.camera_translation = Eigen::Vector3f(0, 0, 0);
		core.camera_up = Eigen::Vector3f(0, 1, 0);
	}
	else {
		core.camera_view_angle = 45;
		Eigen::Quaternionf sceneRot = Eigen::Quaternionf(GetRotation().cast<float>());
		Eigen::Quaternionf headRot = sceneRot * snake->links[SNAKE_LINKS_NUM - 1]->GetGlobalRotation().cast<float>();
		Eigen::Vector3f headTrans = snake->links[SNAKE_LINKS_NUM - 1]->GetGlobalTranslation().cast<float>();
		// LOL
		core.camera_translation = snake->links[0]->GetGlobalRotation().cast<float>() * Eigen::Vector3f(0, 0.5, 0.5) + snake->links[0]->GetGlobalTranslation().cast<float>();
		core.camera_eye = -headTrans * 0.8;
		core.camera_up = headRot * Eigen::Vector3f(0, 1, 0);
	}
}

void Game::ToggleCamera() {
	firstPersonMode = !firstPersonMode;
}

bool Game::UpdateCamera() {
	return firstPersonMode && isActive;
}

void Game::StartLevel() {
	if (playing) {
		std::cout << "Level allready started" << std::endl;
	}
	if (currentLevel > MAX_LEVEL) {
		std::cout << "Congrats you finished the last level. try again (:" << std::endl;
		return;
	}
	std::cout << "starting level " << currentLevel << std::endl;
	for (int i = 0; i < PRICES_NUMBER; i++) {
		prices[i]->SetEaten(i >= currentLevel);
		data(prices[i]->GetId()).Reset();
		snake->Reset(data(snake->GetId()));
		if (i < currentLevel) {
			data(prices[i]->GetId()).set_visible(true, 2);
			prices[i]->ReseedObject(
				data(prices[i]->GetId()),
				GenerateXCord(), /*GenerateYCord()*/0.0, GenerateZCord(),
				GenerateVelocity(), GenerateTravelDistance(),
				GenerateDirection());
		}
	}
	playing = true;
	levelPoints = 0;
}

int Game::GetCurrentLevel() {
	return currentLevel;
}

int Game::GetGamePoints() {
	return gamePoints;
}

int Game::GetLevelPoints() {
	return levelPoints;
}

bool Game::HasMoreLevels() {
	return currentLevel <= MAX_LEVEL;
}

bool Game::FindCollision(int priceIndex) {
	typedef igl::AABB<Eigen::MatrixXd, 3>& AABB_TREE;
	// the stack will hold pairs of trees to check. we use stack for depth search
	std::stack<std::pair<AABB_TREE, AABB_TREE>> boxes_stack;
	boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(snake->GetKTree(), prices[priceIndex]->GetKTree()));
	while (!boxes_stack.empty()) {
		std::pair<AABB_TREE, AABB_TREE> candidate_boxes = boxes_stack.top();
		boxes_stack.pop();
		if (CheckIntersection(candidate_boxes.first.m_box, candidate_boxes.second.m_box, prices[priceIndex]->GetId())) {
			if (candidate_boxes.first.is_leaf() && candidate_boxes.second.is_leaf()) {
				return true;
			}
			else if (candidate_boxes.first.is_leaf()) {
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(candidate_boxes.first, *candidate_boxes.second.m_right));
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(candidate_boxes.first, *candidate_boxes.second.m_left));
			}
			else if (candidate_boxes.second.is_leaf()) {
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_right, candidate_boxes.second));
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_left, candidate_boxes.second));
			}
			else {
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_right, *candidate_boxes.second.m_right));
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_right, *candidate_boxes.second.m_left));
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_left, *candidate_boxes.second.m_right));
				boxes_stack.push(std::pair<AABB_TREE, AABB_TREE>(*candidate_boxes.first.m_left, *candidate_boxes.second.m_left));
			}
		}
	}
	return false;
}

bool Game::CheckIntersection(Eigen::AlignedBox<double, 3> snakeHeadBox, Eigen::AlignedBox<double, 3> priceBox, int priceId) {
	// compute boxes center with respect to scales and translations
	Eigen::Vector3d A_initial_center = snakeHeadBox.center();
	Eigen::Vector3d B_initial_center = priceBox.center();
	Eigen::Vector4d A_expanded_center, B_expanded_center;
	A_expanded_center << A_initial_center, 1;
	B_expanded_center << B_initial_center, 1;
	Eigen::Quaterniond headRotation = snake->links[SNAKE_LINKS_NUM - 1]->GetGlobalRotation();
	Eigen::Vector3d headTranslation = snake->links[SNAKE_LINKS_NUM - 1]->GetGlobalTranslation();
	Eigen::Vector3d a_center = headRotation * A_initial_center + headTranslation;
	Eigen::Vector3d b_center = (data(priceId).MakeTransScaled() * B_expanded_center).block(0, 0, 3, 1);
	// compute boxes dimensions with respect to scales
	Eigen::Vector3d sizes_a = snakeHeadBox.sizes();
	Eigen::Vector3d sizes_b = priceBox.sizes();
	// ccompute boxes axes with respect to rotation
	Eigen::Matrix3d units_a = headRotation.matrix(); // unit vectors of A axes.
	Eigen::Matrix3d units_b = data(priceId).GetRotation(); // unit vectors of B axes.
	Eigen::Matrix3d r_matrix = units_a.transpose() * units_b; // axes dot products matrix
	Eigen::Vector3d T = b_center - a_center;

	// check for A parallel planes
	for (int i = 0; i < 3; i++) {
		double left_side = std::abs(T.dot(units_a.row(i)));
		double right_side = sizes_a(i) + std::abs(sizes_b(0) * r_matrix(i, 0)) + std::abs(sizes_b(1) * r_matrix(i, 1)) + std::abs(sizes_b(2) * r_matrix(i, 2));
		if (left_side > right_side)
			return false;
	}
	// check for B parallel planes
	for (int i = 0; i < 3; i++) {
		double left_side = std::abs(T.dot(units_b.row(i)));
		double right_side = sizes_b(i) + std::abs(sizes_a(0) * r_matrix(0, i)) + std::abs(sizes_a(1) * r_matrix(1, i)) + std::abs(sizes_a(2) * r_matrix(2, i));
		if (left_side > right_side)
			return false;
	}
	// check for edge pairs planes
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			double left_side = std::abs(
				T.dot(units_a.row((i + 2) % 3)) * r_matrix((i + 1) % 3, j) -
				(T.dot(units_a.row((i + 1) % 3)) * r_matrix((i + 2) % 3, j)));

			double right_side =
				std::abs(sizes_a((i + 1) % 3) * r_matrix((i + 2) % 3, j)) +
				std::abs(sizes_a((i + 2) % 3) * r_matrix((i + 1) % 3, j)) +
				std::abs(sizes_b((j + 1) % 3) * r_matrix(i, (j + 2) % 3)) +
				std::abs(sizes_b((j + 2) % 3) * r_matrix(i, (j + 1) % 3));
			if (left_side > right_side) {
				return false;
			}
		}
	}
	return true;
}

void Game::DetectCollisions() {
	for (int i = 0; i < PRICES_NUMBER; i++) {
		if (!prices[i]->IsEaten()) {
			if (FindCollision(i)) {
				std::cout << "You have eaten Price " << prices[i]->GetId() << "! +1 Points" << std::endl;
				levelPoints++;
				gamePoints++;
				prices[i]->SetEaten(true);
				data(prices[i]->GetId()).set_visible(false, 2);
			}
		}
	}
}

bool Game::IsLevelEnded() {
	if (!playing) {
		return false;
	}
	for (int i = 0; i < PRICES_NUMBER; i++) {
		if (!prices[i]->IsEaten()) {
			return false;
		}
	}
	return true;
}

void Game::SetVisibilities() {
	data(snake->GetId()).show_overlay |= 2;
	data(snake->GetId()).show_overlay_depth |= 2;
	for (int i = 0; i < PRICES_NUMBER; i++) {
		data(prices[i]->GetId()).set_visible(!prices[i]->IsEaten(), 2);
		data(prices[i]->GetId()).set_face_based(false);
		data(prices[i]->GetId()).show_texture &= ~2;
	}
}

void Game::DrawSnakeToClosestPrice() {
	int closestIndex = -1;
	double closestDist = -1;
	Eigen::Vector3d closestCenter;

	Eigen::MatrixXd pointsToDraw(0, 3);
	Eigen::MatrixXi edgesToDraw(0, 2);
	for (int i = 0; i < PRICES_NUMBER; i++) {
		//Eigen::Vector3d B_initial_center = priceBox.center();
		if (!prices[i]->IsEaten()) {
			Eigen::Vector4d B_expanded_center;
			B_expanded_center << prices[i]->GetKTree().m_box.center(), 1;
			Eigen::Vector3d b_center = (data(prices[i]->GetId()).MakeTransScaled() * B_expanded_center).block(0, 0, 3, 1);
			double dist = (b_center - snake->links[SNAKE_LINKS_NUM - 1]->GetCurrentEndTipLocation()).norm();
			if (closestIndex == -1 || dist < closestDist) {
				closestIndex = i;
				closestDist = dist;
				closestCenter = b_center;
			}
		}
	}
	if (closestIndex >= 0) {
		pointsToDraw.resize(2, 3);
		edgesToDraw.resize(1, 2);
		pointsToDraw.row(0) = snake->links[SNAKE_LINKS_NUM - 1]->GetCurrentEndTipLocation();
		pointsToDraw.row(1) = closestCenter;
		edgesToDraw.row(0) = Eigen::Vector2i(0, 1);
	}
	data(snake->GetId()).set_edges(pointsToDraw, edgesToDraw, snake->sea_green);
}

void Game::Animate()
{
	if (isActive && playing)
	{
		snake->Animate(*this, data(snake->GetId()));
		for (int i = 0; i < PRICES_NUMBER; i++) {
			if (!prices[i]->IsEaten()) {
				prices[i]->advance(data(prices[i]->GetId()));
			}
		}
		DetectCollisions();
		if (IsLevelEnded()) {
			playing = false;
			std::cout << "Congrats! you have finished level " << currentLevel << " with " << levelPoints << " points. total points: " << gamePoints << std::endl;
			levelPoints = 0;
			currentLevel += 1;
		}
		DrawSnakeToClosestPrice();
	}
}