#include "SnakeLink.h"

SnakeLink::SnakeLink():
	transformation(Eigen::Affine3d::Identity()),
	restPos(Eigen::Quaterniond::Identity())
{
}
