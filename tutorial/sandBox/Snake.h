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
    Snake();
    ~Snake();
    void InitData(igl::opengl::ViewerData& vd);
    void ScaleSnake(igl::opengl::ViewerData& vd);
    void Animate(igl::opengl::glfw::Viewer& scn, igl::opengl::ViewerData& vd);
    void AnimateSnakeAndMove();
    void GetChainCopy(std::vector<SnakeLink>& linksCopy);
    void AlignBodyToHeadNew();
    void AlignBodyToHead();
    void MoveStraightForward();
    //void MoveForward();
    //void UpdateMovement();
    //Eigen::Vector4d GenerateSnakeDestination();
    bool checkIfTargetReached();
    bool checkIfTargetReached(Eigen::Vector3d headPos);
    void CalculateFabrikDestination(Eigen::Vector3d dir);
    void FabrikSolver(std::vector<SnakeLink>& tmpLinks);
    void FabrikSolverNew(std::vector<SnakeLink>& tmpLinks);
    void FabrikSolver();
    //void RefreshFabrikDestination();
    //void FabrikStep(igl::opengl::ViewerData& vd);
    void UpdateSkinning(igl::opengl::ViewerData& vd);
    void CalculateInverseKinematics(igl::opengl::ViewerData& vd);
    void CalculateInverseKinematics();
    void fixZAxisRotation();
    void updateDirection(igl::opengl::ViewerData& vd, int newDirection);
    void updateVelocity(int key);
    void fixZAxisRotation(std::vector<SnakeLink>& chainLinks);
    //void UpdateTips(Eigen::Matrix4d baseTransformations, int firstIndex);
    //Eigen::Matrix4d CalcLinkParentsTransformation(int i);
    //Eigen::Matrix3d CalcLinkParentsRotation(int i);
    void CalculateWeights();
    void CalculateSkeleton(igl::opengl::ViewerData& vd);

    void DrawSkeleton(igl::opengl::ViewerData& vd, Eigen::MatrixXd skelPoints, Eigen::MatrixXi skelEdges);

    void UpdateLinksGlobalTransformations(int firstIndex);

    void UpdateChainGlobalTransformations(int firstIndex, std::vector<SnakeLink>& chainLinks);

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
    int fabCount;

    Eigen::RowVector3d sea_green;
    Eigen::MatrixXd W, C, U, SP;
    Eigen::MatrixXi BE, SE;
    Eigen::VectorXi P;
};