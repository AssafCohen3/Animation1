#include "tutorial/sandBox/Game.h"
#include "Eigen/dense"
#include <functional>
#include <GLFW/glfw3.h>
#include "stack"

#define STEPS_MULTIPLIER	0.1
#define DELTA				0.1

Game::Game() :
	snake(Snake()),
	sea_green(Eigen::RowVector3d(70. / 255., 252. / 255., 167. / 255.)),
	anim_t(0.0),
	anim_t_dir(0.1),
	use_dqs(true),
	recompute(true)
{
#ifndef IGL_VIEWER_VIEWER_QUIET
	//TODO update usage
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

Game::~Game()
{
}

bool Game::load_mesh_from_file(const std::string& mesh_file_name) {
	if (super::load_mesh_from_file(mesh_file_name)) {
		parents.push_back(-1);
		data_list.back().set_visible(true, 2);
		InitMesh();
		return true;
	}
	return false;
}

void Game::Init(const std::string& config)
{
	if (TUTORIAL) {
		InitTest();
		return;
	}
	std::ifstream nameFileout;
	std::string snakeMeshFile;
	std::string snakeTextureFile;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file " << config << std::endl;
	}
	else
	{
		nameFileout >> snakeMeshFile;
		nameFileout >> snakeTextureFile;
		nameFileout.close();
		std::cout << "openning " << snakeMeshFile << std::endl;
		load_mesh_from_file(snakeMeshFile);
		data().image_texture(snakeTextureFile);
		snake.InitData(data());
	}
	MyTranslate(Eigen::Vector3d(0, 0, -10), true);
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	isActive = false;
}

void Game::InitTest()
{
	igl::readOBJ("C:/Users/Lenovo/Documents/animation/EngineForAnimationCourse/tutorial/data/arm.obj", V, F);
	U = V;
	igl::readTGF("C:/Users/Lenovo/Documents/animation/EngineForAnimationCourse/tutorial/data/arm.tgf", C, BE);
	// retrieve parents for forward kinematics
	igl::directed_edge_parents(BE, P);
	RotationList rest_pose;
	igl::directed_edge_orientations(C, BE, rest_pose);
	poses.resize(4, RotationList(4, Eigen::Quaterniond::Identity()));
	// poses[1] // twist
	const Eigen::Quaterniond twist(Eigen::AngleAxisd(igl::PI, Eigen::Vector3d(1, 0, 0)));
	//poses[1][2] = rest_pose[2] * twist * rest_pose[2].conjugate();
	const Eigen::Quaterniond bend(Eigen::AngleAxisd(-igl::PI * 0.7, Eigen::Vector3d(0, 0, 1)));
	poses[3][2] = rest_pose[2] * bend * rest_pose[2].conjugate();

	igl::readDMAT("C:/Users/Lenovo/Documents/animation/EngineForAnimationCourse/tutorial/data/arm-weights.dmat", W);
	igl::lbs_matrix(V, W, M);
	std::cout << "C:" << std::endl;
	std::cout << C << std::endl;
	std::cout << "BE:" << std::endl;
	std::cout << BE << std::endl;
	std::cout << "P:" << std::endl;
	std::cout << P << std::endl;

	data().set_mesh(U, F);
	data().set_edges(C, BE, sea_green);
	parents.push_back(-1);
	data().show_texture ^= 2;
	data().show_overlay_depth = false;
	data().line_width = 1;
}

void Game::keyPressed(int key)
{
	if (TUTORIAL) {
		return;
	}
	switch (key)
	{
	case GLFW_KEY_UP:
	case GLFW_KEY_DOWN:
	case GLFW_KEY_LEFT:
	case GLFW_KEY_RIGHT:
		snake.updateDirection(data(0), key);
		break;
	case 'w':
	case 's':
		snake.updateVelocity(key);
		break;
	default:
		break;
	}
}

void Game::pre_draw()
{
	if (recompute)
	{
		// Find pose interval
		const int begin = (int)floor(anim_t) % poses.size();
		const int end = (int)(floor(anim_t) + 1) % poses.size();
		std::cout << begin << std::endl;
		std::cout << end << std::endl;
		const double t = anim_t - floor(anim_t);

		// Interpolate pose and identity
		RotationList anim_pose(poses[begin].size());
		for (int e = 0; e < poses[begin].size(); e++)
		{
			//anim_pose[e] = poses[begin][e].slerp(t, poses[end][e]);
			anim_pose[e] = poses[end][e];
			std::cout << "************** " << e << " *************" << std::endl;
			std::cout << anim_pose[e].matrix() << std::endl;
		}
		// Propagate relative rotations via FK to retrieve absolute transformations
		RotationList vQ;
		std::vector<Eigen::Vector3d> vT;
		igl::forward_kinematics(C, BE, P, anim_pose, vQ, vT);
		for (int i = 0; i < vQ.size(); i++) {
			std::cout << "************** " << i << " *************" << std::endl;
			std::cout << vQ[i].matrix() << std::endl;
		}
		const int dim = C.cols();
		Eigen::MatrixXd T(BE.rows() * (dim + 1), dim);
		for (int e = 0; e < BE.rows(); e++)
		{
			Eigen::Affine3d a = Eigen::Affine3d::Identity();
			a.translate(vT[e]);
			a.rotate(vQ[e]);
			T.block(e * (dim + 1), 0, dim + 1, dim) =
				a.matrix().transpose().block(0, 0, dim + 1, dim);
		}
		// Compute deformation via LBS as matrix multiplication
		if (use_dqs)
		{
			igl::dqs(V, W, vQ, vT, U);
		}
		else
		{
			U = M * T;
		}

		// Also deform skeleton edges
		Eigen::MatrixXd CT;
		Eigen::MatrixXi BET;
		igl::deform_skeleton(C, BE, T, CT, BET);
		data().set_vertices(U);
		data().set_edges(CT, BET, sea_green);
		data().compute_normals();

		anim_t += anim_t_dir;
	}
}

void Game::InitMesh() {
	data().point_size = 10;
	data().line_width = 2;
	data().show_overlay ^= 2;
	data().show_overlay_depth ^= 2;
	data().set_visible(false, 1);
}
void Game::Animate()
{
	if (isActive)
	{
		snake->Animate(*this, data(snake->get));
	}
}


