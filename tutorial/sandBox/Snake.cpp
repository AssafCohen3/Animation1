#include "Snake.h"

Snake::Snake(int id): 
	velocity(MIN_VELOCITY),
	snakeLength(-1),
	snakeZMax(-1),
	animationDelta(-1),
	directionAngle(-1),
	aligning(false),
	sea_green(Eigen::RowVector3d(70. / 255., 252. / 255., 167. / 255.)),
	id(id)
{

}

Snake::~Snake() {
	for (int i = 0; i < links.size(); i++) {
		delete links[i];
	}
}

void Snake::InitData(igl::opengl::ViewerData& vd) {
	ScaleSnake(vd);
	direction = Eigen::Vector3d(0, 0, 0);
	originalV = Eigen::MatrixXd(vd.V);
	Eigen::Vector3d minVals = vd.V.colwise().minCoeff();
	Eigen::Vector3d maxVals = vd.V.colwise().maxCoeff();
	Eigen::Vector3d meanVals = vd.V.colwise().mean();
	snakeLength = maxVals(2) - minVals(2);
	snakeZMax = maxVals(2);
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		SnakeLink* newLink = new SnakeLink();
		newLink->SetInitialStartTipLocation(Eigen::Vector3d(meanVals(0), meanVals(1), maxVals(2) - i * (snakeLength / SNAKE_LINKS_NUM)));
		newLink->SetInitialEndTipLocation(Eigen::Vector3d(meanVals(0), meanVals(1), maxVals(2) - (i+1) * (snakeLength / SNAKE_LINKS_NUM)));
		links.push_back(newLink);
	}
	UpdateLinksGlobalTransformations(0);
	animationDelta = -1;
	CalculateWeights();
	CalculateHeadKTree(vd);
	DrawLayout(vd);
}

void Snake::ScaleSnake(igl::opengl::ViewerData& vd) {
	Eigen::MatrixXd scaledVs = vd.V;
	Eigen::Vector3d minVals = vd.V.colwise().minCoeff();
	Eigen::Vector3d maxVals = vd.V.colwise().maxCoeff();
	Eigen::Vector3d meanVals = vd.V.colwise().mean();
	snakeLength = maxVals(2) - minVals(2);
	snakeZMax = maxVals(2);
	double requiredLength = SNAKE_LINKS_NUM * SNAKE_LINK_LENGTH;
	double scaleFactor = requiredLength / snakeLength;
	for (int i = 0; i < vd.V.rows(); i++) {
		scaledVs(i, 2) = scaledVs(i, 2) * scaleFactor;
	}
	vd.set_vertices(scaledVs);
	vd.compute_normals();
	snakeLength *= scaleFactor;
	snakeZMax *= scaleFactor;
}

void Snake::Animate(igl::opengl::glfw::Viewer& scn, igl::opengl::ViewerData& vd) {
	if (animationDelta >= 0) {
		AnimateSnakeAndMove();
	}
	else {
		MoveStraightForward();
	}
	UpdateSkinning(vd);
	DrawLayout(vd);
}

void Snake::AnimateSnakeAndMove() {	
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		links[i]->SetLocalRotation(links[i]->GetLocalAnimationStartOrientation().slerp(animationDelta, links[i]->GetLocalAnimationTargetOrientation()));
	}
	Eigen::Vector3d oldDiff = links[SNAKE_LINKS_NUM - 1]->GetCurrentEndTipLocation() - links[0]->GetCurrentStartTipLocation();
	UpdateLinksGlobalTransformations(0);
	Eigen::Vector3d curDiff = links[SNAKE_LINKS_NUM - 1]->GetCurrentEndTipLocation() - links[0]->GetCurrentStartTipLocation();
	Eigen::Quaterniond headRotation = links[SNAKE_LINKS_NUM - 1]->GetGlobalRotation();
	links[0]->Translate(headRotation * Eigen::Vector3d(0, 0, -velocity));
	UpdateLinksGlobalTransformations(0);
	animationDelta += ANIMATION_DELTA_CHANGE;
	if (animationDelta >= 1) {
		if (aligning) {
			for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
				links[i]->SetLocalRotation(links[i]->GetLocalAnimationTargetOrientation());
			}
			aligning = false;
			animationDelta = -1;
		}
		else {
			AlignBodyToHeadNew();
		}
	}
}
void Snake::GetChainCopy(std::vector<SnakeLink>& linksCopy) {
	linksCopy.resize(SNAKE_LINKS_NUM);
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		linksCopy[i] = *links[i];
	}
}

void Snake::AlignBodyToHeadNew() {
	Eigen::Quaterniond currentHeadRot = links[SNAKE_LINKS_NUM - 1]->GetGlobalRotation();
	Eigen::Vector3d toRot = Eigen::Vector3d(0, 0, -snakeLength);
	Eigen::Vector3d rotated = links[SNAKE_LINKS_NUM - 1]->GetCurrentEndTipLocation() + currentHeadRot * toRot;
	fabrikDestination = rotated;
	std::vector<SnakeLink> tmpLinks;
	FabrikSolver(tmpLinks);
	fixZAxisRotation(tmpLinks);
	UpdateChainGlobalTransformations(0, tmpLinks);
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		links[i]->SetLocalAnimationStartOrientation(links[i]->GetLocalRotation());
		links[i]->SetLocalAnimationTargetOrientation(tmpLinks[i].GetLocalRotation());
	}
	animationDelta = 0.0;
	aligning = true;
}

void Snake::MoveStraightForward() {
	Eigen::Quaterniond headRotation = links[SNAKE_LINKS_NUM - 1]->GetGlobalRotation();
	links[0]->Translate(headRotation * Eigen::Vector3d(0, 0, -velocity));
	UpdateLinksGlobalTransformations(0);
}

bool Snake::checkIfTargetReached(Eigen::Vector3d headPos) {
	Eigen::Vector3d finalDistanceV = fabrikDestination - headPos;
	double distance = finalDistanceV.norm();
	return distance <= FABRIK_DELTA;
}

void Snake::CalculateFabrikDestination(Eigen::Vector3d dir) {
	Eigen::Quaterniond currentHeadRot = links[SNAKE_LINKS_NUM - 1]->GetGlobalRotation();
	Eigen::Quaterniond angleRot = Eigen::Quaterniond(Eigen::AngleAxisd(directionAngle / 180.0 * igl::PI, dir));
	Eigen::Quaterniond newDestinationOrientation = currentHeadRot * angleRot;
	Eigen::Vector3d toRot = Eigen::Vector3d(0, 0, -snakeLength);
	Eigen::Vector3d rotated = links[0]->GetCurrentStartTipLocation() + newDestinationOrientation * toRot;
	fabrikDestination = rotated;
}

void Snake::FabrikSolver(std::vector<SnakeLink>& tmpLinks) {
	tmpLinks.resize(SNAKE_LINKS_NUM);
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		tmpLinks[i] = *links[i];
	}
	int c = 0;
	while(!checkIfTargetReached(tmpLinks[SNAKE_LINKS_NUM - 1].GetCurrentEndTipLocation()) && c <= FABRIK_MIN_ITERATIONS) {
		Eigen::Vector3d basePos = tmpLinks[0].GetCurrentStartTipLocation();
		Eigen::Vector3d headPos = tmpLinks[SNAKE_LINKS_NUM - 1].GetCurrentEndTipLocation();
		Eigen::Vector3d destinationPos = fabrikDestination;
		Eigen::Vector3d b = basePos;
		std::vector<Eigen::Vector3d> new_points(SNAKE_LINKS_NUM + 1);
		new_points[SNAKE_LINKS_NUM] = destinationPos;
		for (int i = SNAKE_LINKS_NUM-1; i >= 0; i--) {
			Eigen::Vector3d currentTip = tmpLinks[i].GetCurrentStartTipLocation();
			Eigen::Vector3d nextTip = new_points[i + 1];
			Eigen::Vector3d Vri = nextTip - currentTip;
			double lambdai = SNAKE_LINK_LENGTH / Vri.norm();
			new_points[i] = (1 - lambdai) * nextTip + lambdai * currentTip;
		}
		new_points[0] = b;
		for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
			Eigen::Vector3d currentTip = new_points[i];
			Eigen::Vector3d nextTip = new_points[i + 1];
			Eigen::Vector3d Vri = nextTip - currentTip;
			double lambdai = SNAKE_LINK_LENGTH / Vri.norm();
			new_points[i + 1] = (1 - lambdai) * currentTip + lambdai * nextTip;
		}
		for (int i = 0; i < SNAKE_LINKS_NUM; i++)
		{
			Eigen::Vector3d currentTip = tmpLinks[i].GetCurrentStartTipLocation();
			Eigen::Vector3d nextTip = tmpLinks[i].GetCurrentEndTipLocation();
			Eigen::Vector3d Ve = nextTip - currentTip;
			Eigen::Vector3d Vt = new_points[i + 1] - new_points[i];
			Eigen::Vector3d Vr;
			Vr << Ve.cross(Vt).normalized();
			Eigen::MatrixXd nextLinkRotation = tmpLinks[i].GetGlobalRotation().matrix();
			Eigen::Vector3d eulerAxis = nextLinkRotation.inverse() * Vr;
			double dotProduct = Ve.normalized().dot(Vt.normalized());
			if (abs(dotProduct) > 1) {
				dotProduct = dotProduct < 0 ? -1 : 1;
			}
			double angleToRotate = acos(dotProduct);
			Eigen::Quaterniond rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angleToRotate * STEPS_MULTIPLIER, eulerAxis));
			tmpLinks[i].SetLocalRotation(tmpLinks[i].GetLocalRotation() * rotation);
			UpdateChainGlobalTransformations(i, tmpLinks);
		}
		c += 1;
	}
}

void Snake::FabrikSolverNew(std::vector<SnakeLink>& tmpLinks) {
	tmpLinks.resize(SNAKE_LINKS_NUM);
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		tmpLinks[i] = *links[i];
	}
	while (!checkIfTargetReached(tmpLinks[SNAKE_LINKS_NUM - 1].GetCurrentEndTipLocation())) {
		for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
			Eigen::Vector3d currentTip = tmpLinks[i].GetCurrentStartTipLocation();
			Eigen::Vector3d Vri = fabrikDestination - currentTip;
			double lambdai = SNAKE_LINK_LENGTH / Vri.norm();
			Eigen::Vector3d newTip = (1 - lambdai) * currentTip + lambdai * fabrikDestination;
			Eigen::Vector3d Ve = tmpLinks[i].GetCurrentEndTipLocation() - tmpLinks[i].GetCurrentStartTipLocation();
			Eigen::Vector3d Vt = newTip - tmpLinks[i].GetCurrentStartTipLocation();
			Eigen::Vector3d Vr;
			Vr << Ve.cross(Vt).normalized();
			Eigen::MatrixXd nextLinkRotation = tmpLinks[i].GetGlobalRotation().matrix();
			Eigen::Vector3d eulerAxis = nextLinkRotation.inverse() * Vr;
			double dotProduct = Ve.normalized().dot(Vt.normalized());
			if (abs(dotProduct) > 1) {
				dotProduct = dotProduct < 0 ? -1 : 1;
			}
			double angleToRotate = acos(dotProduct);
			Eigen::Quaterniond rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angleToRotate * STEPS_MULTIPLIER, eulerAxis));
			tmpLinks[i].SetLocalRotation(tmpLinks[i].GetLocalRotation() * rotation);
			UpdateChainGlobalTransformations(i, tmpLinks);
		}
	}
}

void Snake::UpdateSkinning(igl::opengl::ViewerData& vd) {
	Eigen::MatrixXd newVs;
	std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > dQ;
	std::vector<Eigen::Vector3d> dT;
	dQ.resize(SNAKE_LINKS_NUM);
	dT.resize(SNAKE_LINKS_NUM);
	for (int e = 0; e < SNAKE_LINKS_NUM; e++)
	{
		dQ[e] = links[e]->GetGlobalRotation();
		dT[e] = links[e]->GetGlobalTranslation();
	}
	igl::dqs(originalV, weights, dQ, dT, newVs);
	vd.set_vertices(newVs);
	vd.compute_normals();
}

void Snake::CalculateInverseKinematics(igl::opengl::ViewerData& vd) {
	CalculateFabrikDestination(direction);
	std::vector<SnakeLink> tmpLinks;
	FabrikSolver(tmpLinks);
	fixZAxisRotation(tmpLinks);
	UpdateChainGlobalTransformations(0, tmpLinks);
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		links[i]->SetLocalAnimationStartOrientation(links[i]->GetLocalRotation());
		links[i]->SetLocalAnimationTargetOrientation(tmpLinks[i].GetLocalRotation());
	}
	animationDelta = 0.0;
	aligning = false;
}

void Snake::updateDirection(igl::opengl::ViewerData& vd, int newDirection) {
	switch (newDirection)
	{
	case GLFW_KEY_UP:
		direction = Eigen::Vector3d(1, 0, 0);
		directionAngle = ANGLE_CHANGE_SIZE;
		break;
	case GLFW_KEY_DOWN:
		direction = Eigen::Vector3d(1, 0, 0);
		directionAngle = -ANGLE_CHANGE_SIZE;
		break;
	case GLFW_KEY_LEFT:
		direction = Eigen::Vector3d(0, 1, 0);
		directionAngle = ANGLE_CHANGE_SIZE;
		break;
	case GLFW_KEY_RIGHT:
		direction = Eigen::Vector3d(0, 1, 0);
		directionAngle = -ANGLE_CHANGE_SIZE;
		break;
	}
	CalculateInverseKinematics(vd);
}

void Snake::updateVelocity(int key)
{
	switch (key) {
	case 'w':
		velocity += VELOCITY_INCREMENT_SIZE;
		break;
	case 's':
		velocity = std::max(MIN_VELOCITY, velocity - VELOCITY_INCREMENT_SIZE);
		break;
	}
}

void Snake::fixZAxisRotation(std::vector<SnakeLink>& chainLinks) {
	for (int i = 0; i < SNAKE_LINKS_NUM; i++)
	{
		Eigen::Quaterniond rotationQuat = chainLinks[i].GetLocalRotation();
		Eigen::Vector3d rotationAxis = Eigen::Vector3d(rotationQuat.x(), rotationQuat.y(), rotationQuat.z());
		Eigen::Vector3d directionAxis = Eigen::Vector3d(0, 0, 1);
		double dotProd = directionAxis.dot(rotationAxis);
		Eigen::Vector3d projection = dotProd * directionAxis;
		Eigen::Quaterniond twist = Eigen::Quaterniond(rotationQuat.w(), 
			projection(0), projection(1), projection(2));
		twist.normalize();
		if (dotProd < 0.0) {
			twist.x() = -twist.x();
			twist.y() = -twist.y();
			twist.z() = -twist.z();
			twist.w() = -twist.w();
		}
		Eigen::Quaterniond swing = rotationQuat * twist.conjugate();
		chainLinks[i].SetLocalRotation(swing);
		if (i < chainLinks.size() - 1) {
			chainLinks[i + 1].SetLocalRotation(chainLinks[i + 1].GetLocalRotation() * twist);
		}
	}
}

void Snake::CalculateWeights()
{	
	weights.resize(originalV.rows(), SNAKE_LINKS_NUM);
	for (int i = 0; i < originalV.rows(); i++) {
		double z = originalV(i, 2);
		double test = snakeZMax - originalV(i, 2);
		double loc = ((snakeZMax - originalV(i, 2)) * SNAKE_LINKS_NUM) / snakeLength;
		Eigen::RowVectorXd vWeight = Eigen::RowVectorXd(SNAKE_LINKS_NUM).setZero();
		int floorJoint = std::min((double)(SNAKE_LINKS_NUM - 1), floor(loc));
		double floorWeight = 1 - (loc - floorJoint);
		double ceilWeight = 1 - floorWeight;
		if (floorWeight >= 0.9 && floorJoint < SNAKE_LINKS_NUM - 1) {
			floorWeight -= 0.2;
			if (floorJoint > 0) {
				ceilWeight = (1 - floorWeight) * (1 - ((floorJoint - loc) / 2));
				vWeight(floorJoint - 1) = (1 - floorWeight - ceilWeight);
			}
			else{
				ceilWeight += 0.2;
			}
		}
		else if (ceilWeight >= 0.9 && floorJoint < SNAKE_LINKS_NUM - 1) {
			ceilWeight -= 0.2;
			if (floorJoint + 1 < SNAKE_LINKS_NUM - 1) {
				floorWeight = (1 - ceilWeight) * (1 - ((loc - floorJoint) / 2));
				vWeight(floorJoint + 2) = (1 - floorWeight - ceilWeight);
			}
			else {
				floorWeight += 0.2;
			}
		}
		if (floorJoint == SNAKE_LINKS_NUM - 1) {
			vWeight(floorJoint) = 1;
		}
		else {
			vWeight(floorJoint) = floorWeight;
			vWeight(floorJoint + 1) = ceilWeight;
		}
		weights.row(i) = vWeight;
	}
}

void Snake::DrawLayout(igl::opengl::ViewerData& vd) {
	if (4 == 4) {
		// Skip drawing layout as in game its not very useful
		return;
	}
	Eigen::MatrixXd points(SNAKE_LINKS_NUM + 1 + 8, 3);
	Eigen::MatrixXi edges(SNAKE_LINKS_NUM + 12, 2);
	Eigen::MatrixXd edgesColors(SNAKE_LINKS_NUM + 12, 3);
	points.row(0) = links[0]->GetCurrentStartTipLocation();
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		points.row(i+1) = links[i]->GetCurrentEndTipLocation();
		edges.row(i) = Eigen::Vector2i(i, i + 1);
		edgesColors.row(i) = sea_green;
	}
	for (int i = 0; i < 8; i++) {
		points.row(SNAKE_LINKS_NUM + i + 1) = links[SNAKE_LINKS_NUM - 1]->GetGlobalRotation() * kTree.m_box.corner(static_cast<Eigen::AlignedBox3d::CornerType>(i)) + links[SNAKE_LINKS_NUM - 1]->GetGlobalTranslation();
	}
	// Edges of the bounding box
	Eigen::MatrixXi E_box(12, 2);
	edges.row(SNAKE_LINKS_NUM) = Eigen::Vector2i(SNAKE_LINKS_NUM + 1, SNAKE_LINKS_NUM + 2);
	edges.row(SNAKE_LINKS_NUM+1) = Eigen::Vector2i(SNAKE_LINKS_NUM + 2, SNAKE_LINKS_NUM + 4);
	edges.row(SNAKE_LINKS_NUM+2) = Eigen::Vector2i(SNAKE_LINKS_NUM + 3, SNAKE_LINKS_NUM + 4);
	edges.row(SNAKE_LINKS_NUM+3) = Eigen::Vector2i(SNAKE_LINKS_NUM + 3, SNAKE_LINKS_NUM + 1);
	edges.row(SNAKE_LINKS_NUM+4) = Eigen::Vector2i(SNAKE_LINKS_NUM + 5, SNAKE_LINKS_NUM + 6);
	edges.row(SNAKE_LINKS_NUM+5) = Eigen::Vector2i(SNAKE_LINKS_NUM + 6, SNAKE_LINKS_NUM + 8);
	edges.row(SNAKE_LINKS_NUM+6) = Eigen::Vector2i(SNAKE_LINKS_NUM + 7, SNAKE_LINKS_NUM + 8);
	edges.row(SNAKE_LINKS_NUM+7) = Eigen::Vector2i(SNAKE_LINKS_NUM + 7, SNAKE_LINKS_NUM + 5);
	edges.row(SNAKE_LINKS_NUM+8) = Eigen::Vector2i(SNAKE_LINKS_NUM + 3, SNAKE_LINKS_NUM + 7);
	edges.row(SNAKE_LINKS_NUM+9) = Eigen::Vector2i(SNAKE_LINKS_NUM + 1, SNAKE_LINKS_NUM + 5);
	edges.row(SNAKE_LINKS_NUM+10) = Eigen::Vector2i(SNAKE_LINKS_NUM + 2, SNAKE_LINKS_NUM + 6);
	edges.row(SNAKE_LINKS_NUM+11) = Eigen::Vector2i(SNAKE_LINKS_NUM + 8, SNAKE_LINKS_NUM + 4);
	vd.set_edges(points, edges, edgesColors);
	vd.set_points(points, Eigen::RowVector3d(1, 0, 0));
}

void Snake::CalculateHeadKTree(igl::opengl::ViewerData& vd){
	Eigen::VectorXi is_selected = (originalV.col(2).array() <= links[SNAKE_LINKS_NUM - 1]->GetInitialStartTipLocation()(2)).cast<int>();
	Eigen::MatrixXd headPoints(is_selected.sum(), 3);
	Eigen::MatrixXi headFaces;
	std::vector<Eigen::Vector3i> headFacesList;
	std::map<int, int> pointsMapping;
	int foundedPoints = 0;
	for (int i = 0; i < vd.V.rows(); i++) {
		if (is_selected[i]) {
			headPoints.row(foundedPoints) = vd.V.row(i);
			pointsMapping[i] = foundedPoints;
			foundedPoints++;
		}
	}
	for (int i = 0; i < vd.F.rows(); i++) {
		if (is_selected[vd.F(i, 0)] && is_selected[vd.F(i, 1)] && is_selected[vd.F(i, 2)]) {
			headFacesList.push_back(Eigen::Vector3i(pointsMapping[vd.F(i, 0)], pointsMapping[vd.F(i, 1)], pointsMapping[vd.F(i, 2)]));
		}
	}
	headFaces.resize(headFacesList.size(), 3);
	for (int i = 0; i < headFacesList.size(); i++) {
		headFaces.row(i) = headFacesList[i];
	}
	kTree.init(headPoints, headFaces);
}

void Snake::UpdateLinksGlobalTransformations(int firstIndex) {
	for (int i = firstIndex; i < SNAKE_LINKS_NUM; i++) {
		if (i == 0) {
			links[i]->ApplyTransformations(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
		}
		else {
			links[i]->ApplyTransformations(links[i - 1]->GetGlobalRotation(), links[i - 1]->GetGlobalTranslation());
		}
	}
}

void Snake::UpdateChainGlobalTransformations(int firstIndex, std::vector<SnakeLink>& chainLinks) {
	for (int i = firstIndex; i < chainLinks.size(); i++) {
		if (i == 0) {
			chainLinks[i].ApplyTransformations(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
		}
		else {
			chainLinks[i].ApplyTransformations(chainLinks[i - 1].GetGlobalRotation(), chainLinks[i - 1].GetGlobalTranslation());
		}
	}
}

void Snake::Reset(igl::opengl::ViewerData& vd) {
	velocity = MIN_VELOCITY;
	aligning = false;
	direction = Eigen::Vector3d(0, 0, 0);
	animationDelta = -1;
	directionAngle = -1;
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		links[i]->ResetLocals();
	}
	UpdateLinksGlobalTransformations(0);
	animationDelta = -1;
	UpdateSkinning(vd);
	DrawLayout(vd);
}
