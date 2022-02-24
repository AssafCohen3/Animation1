#pragma once
#include <Eigen/core>
#include "set"
#include "vector"
#include "igl/opengl/ViewerData.h"
#include "igl/AABB.h"
#include "igl/opengl/glfw/Viewer.h"
#include "SnakeLink.h"
#include <GLFW/glfw3.h>
#include "igl/dqs.h"
#include "igl/forward_kinematics.h"
#include "igl/directed_edge_orientations.h"
#include "igl/deform_skeleton.h"
#include "igl/opengl/Movable.h"
#include "Eigen/core"
#include "igl/PI.h"

#define STEPS_MULTIPLIER	    0.1
#define SNAKE_LINK_LENGTH       0.5
#define SNAKE_LINKS_NUM         16
#define MIN_VELOCITY            0.1
#define VELOCITY_INCREMENT_SIZE 0.1
#define ANGLE_CHANGE_SIZE       90
#define ANGLE_THRESHOLD         0.0001
#define FABRIK_MIN_ITERATIONS   20
#define FABRIK_DELTA            1.0
#define FABRIK_ANGLE_STEP       1.0
#define ANIMATION_DELTA_CHANGE  0.02

class Snake {
public:
    Snake(int id);
    ~Snake();
    void InitData(igl::opengl::ViewerData& vd);
    void ScaleSnake(igl::opengl::ViewerData& vd);
    void Animate(igl::opengl::glfw::Viewer& scn, igl::opengl::ViewerData& vd);
    void AnimateSnakeAndMove();
    void GetChainCopy(std::vector<SnakeLink>& linksCopy);
    void AlignBodyToHeadNew();
    void MoveStraightForward();
    bool checkIfTargetReached(Eigen::Vector3d headPos);
    void CalculateFabrikDestination(Eigen::Vector3d dir);
    void FabrikSolver(std::vector<SnakeLink>& tmpLinks);
    void FabrikSolverNew(std::vector<SnakeLink>& tmpLinks);
    void UpdateSkinning(igl::opengl::ViewerData& vd);
    void CalculateInverseKinematics(igl::opengl::ViewerData& vd);
    void updateDirection(igl::opengl::ViewerData& vd, int newDirection);
    void updateVelocity(int key);
    void fixZAxisRotation(std::vector<SnakeLink>& chainLinks);
    void CalculateWeights();
    void DrawLayout(igl::opengl::ViewerData& vd);
    void UpdateLinksGlobalTransformations(int firstIndex);
    void UpdateChainGlobalTransformations(int firstIndex, std::vector<SnakeLink>& chainLinks);
    void CalculateHeadKTree(igl::opengl::ViewerData& vd);
    void Reset(igl::opengl::ViewerData& vd);

    int GetId() { return id; };
    void SetId(int id) { this->id = id; };

    igl::AABB<Eigen::MatrixXd, 3>& GetKTree() { return kTree; };

    int id;
    std::vector<SnakeLink*> links;
    Eigen::Vector3d direction;
    Eigen::MatrixXd weights;
    Eigen::MatrixXd originalV;
    Eigen::Vector3d fabrikDestination;
    double snakeLength;
    double snakeZMax;
    double velocity;
    double animationDelta;
    double directionAngle;
    bool aligning;
    igl::AABB<Eigen::MatrixXd, 3> kTree;
    Eigen::RowVector3d sea_green;
};