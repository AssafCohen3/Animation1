#include "Snake.h"


Snake::Snake(): 
	velocity(MIN_VELOCITY),
	sea_green(Eigen::RowVector3d(70. / 255., 252. / 255., 167. / 255.)),
	fabCount(0),
	aligning(false)
{

}

Snake::~Snake() {
	for (int i = 0; i < links.size(); i++) {
		delete links.at(i);
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
	//double requiredLength = SNAKE_LINKS_NUM * SNAKE_LINK_LENGTH;
	//double scaleFactor = requiredLength / snakeLength;
	//Eigen::Vector4d scaleFull = Eigen::Vector4d(1, 1, scaleFactor, 1);
	//vd.MyScale(scaleFull.head(3));
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		SnakeLink* newLink = new SnakeLink();
		//newLink->MyScale(scaleLink.head(3));
		newLink->SetInitialStartTipLocation(Eigen::Vector3d(meanVals(0), meanVals(1), maxVals(2) - i * (snakeLength / SNAKE_LINKS_NUM)));
		newLink->SetInitialEndTipLocation(Eigen::Vector3d(meanVals(0), meanVals(1), maxVals(2) - (i+1) * (snakeLength / SNAKE_LINKS_NUM)));
		links.push_back(newLink);
	}
	UpdateLinksGlobalTransformations(0);
	animationDelta = -1;
	CalculateWeights();
	//std::cout << weights << std::endl;
	// retrieve parents for forward kinematics
	CalculateSkeleton(vd);
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		//std::cout << "********** " << i << " **********" << std::endl;
		//std::cout << links[i]->GetRotation().matrix() << std::endl;
		//std::cout << links[i]->MakeTransd() << std::endl;
	}
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
	//FabrikStep(vd);
	if (animationDelta >= 0) {
		AnimateSnakeAndMove();
	}
	else {
		MoveStraightForward();
	}
	UpdateSkinning(vd);
	DrawSkeleton(vd, SP, SE);
	//fixZAxisRotation();
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
	//links[0]->Translate(headRotation* ((curDiff - oldDiff) + Eigen::Vector3d(0, 0, -velocity)));
	UpdateLinksGlobalTransformations(0);
	animationDelta += ANIMATION_DELTA_CHANGE;
	if (animationDelta >= 1) {
		if (aligning) {
			std::cout << "Finished Alignment" << std::endl;
			for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
				links[i]->SetLocalRotation(links[i]->GetLocalAnimationTargetOrientation());
			}
			aligning = false;
			animationDelta = -1;
		}
		else {
			std::cout << "Finished animation" << std::endl;
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
void Snake::AlignBodyToHead() {
	std::vector<SnakeLink> tmpLinks;
	GetChainCopy(tmpLinks);
	
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		Eigen::Vector3d curDir = tmpLinks[i].GetCurrentEndTipLocation() - tmpLinks[i].GetCurrentStartTipLocation();
		Eigen::Vector3d newDir = tmpLinks[i+1].GetCurrentEndTipLocation() - tmpLinks[i].GetCurrentStartTipLocation();
		Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(curDir, newDir);
		tmpLinks[i].SetLocalRotation(tmpLinks[i].GetLocalRotation() * rotation.conjugate());
		//if (i < SNAKE_LINKS_NUM - 1) {
		//	tmpLinks[i + 1].SetLocalRotation(rotation.conjugate() * tmpLinks[i + 1].GetLocalRotation());
		//}
		UpdateChainGlobalTransformations(i, tmpLinks);
		Eigen::Vector3d curDir2 = tmpLinks[i].GetCurrentEndTipLocation() - tmpLinks[i].GetCurrentStartTipLocation();
		Eigen::Vector3d newDir2 = tmpLinks[SNAKE_LINKS_NUM - 1].GetCurrentEndTipLocation() - tmpLinks[i].GetCurrentStartTipLocation();
		Eigen::Quaterniond rotation2 = Eigen::Quaterniond::FromTwoVectors(curDir2, newDir2);
		//std::cout << rotation.matrix() << std::endl;
		//std::cout << rotation2.matrix() << std::endl;
	}
	fixZAxisRotation(tmpLinks);
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		links[i]->SetLocalAnimationStartOrientation(links[i]->GetLocalRotation());
		links[i]->SetLocalAnimationTargetOrientation(tmpLinks[i].GetLocalRotation());
	}
	//animationDelta = -1;
	animationDelta = 0;
	aligning = true;
}

void Snake::MoveStraightForward() {
	Eigen::Quaterniond headRotation = links[SNAKE_LINKS_NUM - 1]->GetGlobalRotation();
	links[0]->Translate(headRotation * Eigen::Vector3d(0, 0, -velocity));
	UpdateLinksGlobalTransformations(0);
}

//Eigen::Vector4d Snake::GenerateSnakeDestination() {
//	Eigen::Quaterniond headRotation = Eigen::Quaterniond(CalcLinkParentsRotation(SNAKE_LINKS_NUM));
//	Eigen::Quaterniond rotationMat = Eigen::Quaterniond(Eigen::AngleAxisd((ANGLE_CHANGE_SIZE / 180.0) * igl::PI, direction.head(3)));
//	//Eigen::Quaterniond rotationMat = Eigen::Quaterniond(Eigen::AngleAxisd((ANGLE_CHANGE_SIZE / 180.0), headRotation * direction * headRotation.conjugate() *
//	//	direction.head(3)).matrix());
//	//Eigen::Matrix3d rotationMat = Eigen::AngleAxisd((ANGLE_CHANGE_SIZE / 180.0), direction.head(3)).matrix();
//	Eigen::Quaterniond globalDirection = headRotation * rotationMat * headRotation.conjugate();// headRotation* rotationMat;
//	Eigen::Vector3d amt = globalDirection * Eigen::Vector3d(0, 0, -3);
//	Eigen::Vector4d destinationPos = links[SNAKE_LINKS_NUM - 1]->getTip() + Eigen::Vector4d(amt(0), amt(1), amt(2), 0);
//	return destinationPos;
//}

bool Snake::checkIfTargetReached(Eigen::Vector3d headPos) {
	Eigen::Vector3d finalDistanceV = fabrikDestination - headPos;
	double distance = finalDistanceV.norm();
	//std::cout << distance << std::endl;
	if (distance <= FABRIK_DELTA) {
		std::cout << "reached fabrik. distance: " << distance << std::endl;
		return true;
	}
	return false;
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
	//std::vector<Eigen::Quaterniond> tmpGlobalRotations(SNAKE_LINKS_NUM);
	//std::vector<Eigen::Quaterniond> tmpLocalRotations(SNAKE_LINKS_NUM);
	//std::vector<Eigen::Vector3d> tmpLocalTranslation(SNAKE_LINKS_NUM);
	//std::vector<Eigen::Vector3d> tmpStartLocations(SNAKE_LINKS_NUM);
	//std::vector<Eigen::Vector3d> tmpEndLocations(SNAKE_LINKS_NUM);
	tmpLinks.resize(SNAKE_LINKS_NUM);
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		tmpLinks[i] = *links[i];
		//tmpGlobalRotations[i] = links[i]->GetGlobalRotation();
		//tmpLocalRotations[i] = links[i]->GetLocalRotation();
		//tmpStartLocations[i] = links[i]->GetCurrentStartTipLocation();
		//tmpEndLocations[i] = links[i]->GetCurrentEndTipLocation();
		//tmpLocalTranslation[i] = links[i]->GetLocalTranslation();
		//links[i]->SetLocalAnimationStartOrientation(links[i]->GetLocalRotation());
		//links[i]->GetLocalAnimationTargetOrientation(links[i]->GetLocalRotation());
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
		//Eigen::Vector4d advancePoint = new_points[0];
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
			//Eigen::MatrixXd currentLinkRotation = tmpro;
			Eigen::MatrixXd nextLinkRotation = tmpLinks[i].GetGlobalRotation().matrix();
			Eigen::Vector3d eulerAxis = nextLinkRotation.inverse() * Vr;
			double dotProduct = Ve.normalized().dot(Vt.normalized());
			if (abs(dotProduct) > 1) {
				dotProduct = dotProduct < 0 ? -1 : 1;
			}
			double angleToRotate = acos(dotProduct);
			/*if (angleToRotate < ANGLE_THRESHOLD) {
				continue;
			}*/
			//links[i]->rotate(eulerAxis.head(3), angleToRotate * STEPS_MULTIPLIER);
			Eigen::Quaterniond rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angleToRotate * STEPS_MULTIPLIER, eulerAxis));
			tmpLinks[i].SetLocalRotation(tmpLinks[i].GetLocalRotation() * rotation);
			//tmpLocalRotations[i] = tmpLocalRotations[i] * rotation;
			//tmpGlobalRotations[i] = i == 0 ? tmpLocalRotations[i] : tmpGlobalRotations[i - 1] * tmpLocalRotations[i];
			//tmpStartLocations[i] = rotation * tmpStartLocations[i] * rotation.conjugate();
			//tmpEndLocations[i] = rotation * tmpEndLocations[i] * rotation.conjugate();
			//links[i]->rotate(rotation);
			UpdateChainGlobalTransformations(i, tmpLinks);
			//UpdateTips(currentLinkRotation, i + 1);
		}
		//links[0]->translate((direction.MakeTransd() * (new_points[0] - basePos)).head(3) * velocity);
		fabCount += 0;
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
			//Eigen::MatrixXd currentLinkRotation = tmpro;
			Eigen::MatrixXd nextLinkRotation = tmpLinks[i].GetGlobalRotation().matrix();
			Eigen::Vector3d eulerAxis = nextLinkRotation.inverse() * Vr;
			double dotProduct = Ve.normalized().dot(Vt.normalized());
			if (abs(dotProduct) > 1) {
				dotProduct = dotProduct < 0 ? -1 : 1;
			}
			double angleToRotate = acos(dotProduct);
			/*if (angleToRotate < ANGLE_THRESHOLD) {
				continue;
			}*/
			//links[i]->rotate(eulerAxis.head(3), angleToRotate * STEPS_MULTIPLIER);
			Eigen::Quaterniond rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angleToRotate * STEPS_MULTIPLIER, eulerAxis));
			tmpLinks[i].SetLocalRotation(tmpLinks[i].GetLocalRotation() * rotation);
			//tmpLocalRotations[i] = tmpLocalRotations[i] * rotation;
			//tmpGlobalRotations[i] = i == 0 ? tmpLocalRotations[i] : tmpGlobalRotations[i - 1] * tmpLocalRotations[i];
			//tmpStartLocations[i] = rotation * tmpStartLocations[i] * rotation.conjugate();
			//tmpEndLocations[i] = rotation * tmpEndLocations[i] * rotation.conjugate();
			//links[i]->rotate(rotation);
			UpdateChainGlobalTransformations(i, tmpLinks);
			//UpdateTips(currentLinkRotation, i + 1);
		}
	}
}

// Dual quaternion skinning
//
// Inputs:
//   V  #V by 3 list of rest positions
//   W  #W by #C list of weights
//   vQ  #C list of rotation quaternions 
//   vT  #C list of translation vectors
// Outputs:
//   U  #V by 3 list of new positions
void Snake::UpdateSkinning(igl::opengl::ViewerData& vd) {
	const int dim = C.cols();
	Eigen::MatrixXd newVs;
	Eigen::MatrixXd T(BE.rows() * (dim + 1), dim);
	std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > dQ;
	std::vector<Eigen::Vector3d> dT;
	dQ.resize(SNAKE_LINKS_NUM);
	dT.resize(SNAKE_LINKS_NUM);
	for (int e = 0; e < BE.rows(); e++)
	{
		Eigen::Affine3d a = Eigen::Affine3d::Identity();
		a.translate(links[e]->GetGlobalTranslation());
		a.rotate(links[e]->GetGlobalRotation());
		T.block(e * (dim + 1), 0, dim + 1, dim) =
			a.matrix().transpose().block(0, 0, dim + 1, dim);
		dQ[e] = links[e]->GetGlobalRotation();
		dT[e] = links[e]->GetGlobalTranslation();
	}
	igl::dqs(originalV, weights, dQ, dT, newVs);
	igl::deform_skeleton(C, BE, T, SP, SE);
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
	/*if (fabCount < FABRIK_MIN_ITERATIONS) {
		std::cout << "did not finished movement." << std::endl;
	}*/
	//animationDelta = "ssss";
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

	//Eigen::Quaterniond headRotation = Eigen::Quaterniond(CalcLinkParentsRotation(SNAKE_LINKS_NUM));
	//Eigen::Quaterniond rotationMat = Eigen::Quaterniond(Eigen::AngleAxisd((ANGLE_CHANGE_SIZE / 180.0) * igl::PI, direction.head(3)));
	////Eigen::Quaterniond rotationMat = Eigen::Quaterniond(Eigen::AngleAxisd((ANGLE_CHANGE_SIZE / 180.0), headRotation * direction * headRotation.conjugate() *
	////	direction.head(3)).matrix());
	////Eigen::Matrix3d rotationMat = Eigen::AngleAxisd((ANGLE_CHANGE_SIZE / 180.0), direction.head(3)).matrix();
	//Eigen::Quaterniond globalDirection = rotationMat;// headRotation* rotationMat;
	//links[SNAKE_LINKS_NUM - 2]->rotate(headRotation.inverse() * globalDirection);
	//direction = Eigen::Vector4d(0, 0, 0, 0);
	CalculateInverseKinematics(vd);
	DrawSkeleton(vd, SP, SE);
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
		//Eigen::Vector3d p = vt * (q.x() * vt[0] + q.y() * vt[1] + q.z() * vt[2]);
		//twist = Eigen::Quaterniond(q.w(), p[0], p[1], p[2]);
		//twist.normalize();
		//swing = q * twist.conjugate();
		Eigen::Quaterniond rotationQuat = chainLinks[i].GetLocalRotation();
		Eigen::Vector3d rotationAxis = Eigen::Vector3d(rotationQuat.x(), rotationQuat.y(), rotationQuat.z());
		//Vector3d rotationAxis = new Vector3d(rotation.x, rotation.y, rotation.z);
		Eigen::Vector3d directionAxis = Eigen::Vector3d(0, 0, 1);
		double dotProd = directionAxis.dot(rotationAxis);
		// Shortcut calculation of `projection` requires `direction` to be normalized
		Eigen::Vector3d projection = dotProd * directionAxis;
		Eigen::Quaterniond twist = Eigen::Quaterniond(rotationQuat.w(), 
			projection(0), projection(1), projection(2));
		twist.normalize();
		if (dotProd < 0.0) {
			// Ensure `twist` points towards `direction`
			twist.x() = -twist.x();
			twist.y() = -twist.y();
			twist.z() = -twist.z();
			twist.w() = -twist.w();
			// Rotation angle `twist.angle()` is now reliable
		}
		Eigen::Quaterniond swing = rotationQuat * twist.conjugate();
		chainLinks[i].SetLocalRotation(swing);
		if (i < chainLinks.size() - 1) {
			chainLinks[i + 1].SetLocalRotation(chainLinks[i + 1].GetLocalRotation() * twist);
		}
	}
}

//Eigen::Matrix4d Snake::CalcLinkParentsTransformation(int i) {
//	Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
//	for (int j = 0; j < i; j++) {
//		transformation *= links[j]->MakeTransd();
//	}
//	return transformation;
//}
//
//Eigen::Matrix3d Snake::CalcLinkParentsRotation(int i) {
//	Eigen::Matrix3d transformation = Eigen::Matrix3d::Identity();
//	for (int j = 0; j < i; j++) {
//		transformation *= links[j]->GetRotation().matrix();
//	}
//	return transformation;
//}

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

void Snake::CalculateSkeleton(igl::opengl::ViewerData& vd)
{
	C.resize(SNAKE_LINKS_NUM+1, 3);
	BE.resize(SNAKE_LINKS_NUM, 2);
	P.resize(SNAKE_LINKS_NUM);
	
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		C.row(i) = links[i]->GetInitialStartTipLocation();
		//C(i, 2) = C(i, 2) * scaleFactor;
		BE.row(i) = Eigen::Vector2i(i, i + 1);
		P(i) = i - 1;
	}
	C.row(SNAKE_LINKS_NUM) = links[SNAKE_LINKS_NUM - 1]->GetInitialEndTipLocation();
	std::cout << "C:" << std::endl;
	std::cout << C << std::endl;
	std::cout << "BE:" << std::endl;
	std::cout << BE << std::endl;
	std::cout << "P:" << std::endl;
	std::cout << P << std::endl;
	//std::cout << "W:" << std::endl;
	//std::cout << weights << std::endl;
	std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > restPoses;
	igl::directed_edge_orientations(C, BE, restPoses);
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		//links[i]->SetRestPose(restPoses[i]);
		//std::cout << "++++++++++ " << i << " ++++++++++" << std::endl;
		//std::cout << restPoses[i].matrix() << std::endl;
	}
	SP = C;
	SE = BE;
	DrawSkeleton(vd, C, BE);
}

void Snake::DrawSkeleton(igl::opengl::ViewerData& vd, Eigen::MatrixXd skelPoints, Eigen::MatrixXi skelEdges) {
	Eigen::MatrixXd minPoints(SNAKE_LINKS_NUM + 1, 3);
	Eigen::MatrixXi mineEdges(SNAKE_LINKS_NUM + 1, 2);
	minPoints.row(0) = links[0]->GetCurrentStartTipLocation();
	for (int i = 0; i < SNAKE_LINKS_NUM; i++) {
		minPoints.row(i+1) = links[i]->GetCurrentEndTipLocation();
		mineEdges.row(i) = Eigen::Vector2i(i, i + 1);
	}
	mineEdges.row(mineEdges.rows() - 1) = Eigen::Vector2i(0, SNAKE_LINKS_NUM);
	//Eigen::Vector3d destinationPoint = GenerateSnakeDestination().head(3);
	//Eigen::Vector2i destinationEdge = Eigen::Vector2i(skelEdges(skelEdges.rows() - 1, 1), skelPoints.rows());
	//std::cout << destinationPoint << std::endl;
	//skelPoints.conservativeResize(skelPoints.rows() + 1, Eigen::NoChange);
	//skelEdges.conservativeResize(skelEdges.rows() + 2, Eigen::NoChange);
	//skelPoints.row(skelPoints.rows() - 2) = destinationPoint;
	//skelEdges.row(skelEdges.rows() - 1) = destinationEdge;
	//minPoints.row(minPoints.rows() - 1) = fabrikDestination;
	//skelPoints.row(skelPoints.rows() - 1) = Eigen::Vector3d(5, 0, 0);
	//skelPoints.row(skelPoints.rows() - 1) = Eigen::Vector3d(0, 5, 0);
	/*std::cout << " *************** dir **************" << std::endl;
	std::cout << skelPoints << std::endl;
	std::cout << skelEdges << std::endl;*/
	vd.set_edges(minPoints, mineEdges, sea_green);
	vd.set_points(minPoints, Eigen::RowVector3d(1, 0, 0));
}

void Snake::UpdateLinksGlobalTransformations(int firstIndex) {
	/*
	        vQ[b] = vQ[p] * dQ[b];
        const Vector3d r = C.row(BE(b,0)).transpose();
        vT[b] = vT[p] - vQ[b]*r + vQ[p]*(r + dT[b]);
*/
	// Global transformations of previous link should be computed allready
	for (int i = firstIndex; i < SNAKE_LINKS_NUM; i++) {
		if (i == 0) {
			links[i]->ApplyTransformations(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
		}
		else {
			links[i]->ApplyTransformations(links[i - 1]->GetGlobalRotation(), links[i - 1]->GetGlobalTranslation());
		}
		//baseTransformations *= links[i]->MakeTransd();
		//links[i]->setTip(baseTransformations * links[i]->getRestEndPose());
	}
}

//void UpdateChainGlobalTransformations(int firstIndex, std::vector<Eigen::Quaterniond>& localRotations, std::vector<Eigen::Quaterniond>& globalRotations,
//	std::vector<Eigen::Vector3d>& localTranslations,
//	std::vector<Eigen::Vector3d>& startLocations, std::vector<Eigen::Vector3d> endLocations) {
//	/*
//			vQ[b] = vQ[p] * dQ[b];
//		const Vector3d r = C.row(BE(b,0)).transpose();
//		vT[b] = vT[p] - vQ[b]*r + vQ[p]*(r + dT[b]);
//*/
//// Global transformations of previous link should be computed allready
//	Eigen::Quaterniond curRotation = firstIndex == 0 ? Eigen::Quaterniond::Identity() : globalRotations[firstIndex-1];
//	//Eigen::Quaterniond cu
//	for (int i = firstIndex; i < localRotations.size(); i++) {
//		globalRotations[i] = curRotation * localRotations[i];
//
//		Eigen::Quaterniond toRotate = Eigen::Quaterniond::FromTwoVectors(globalRotations);
//		Eigen::Vector3d globalTranslationVector = t - globalRotation * initialEndTipLocation + rot * (initialEndTipLocation + localTranslationVector);
//		currentStartTipLocation = ApplyToPoint(initialStartTipLocation);
//		currentEndTipLocation = ApplyToPoint(initialEndTipLocation);
//
//		links[i]->ApplyTransformations(links[i - 1]->GetGlobalRotation(), links[i - 1]->GetGlobalTranslation());
//		//baseTransformations *= links[i]->MakeTransd();
//		//links[i]->setTip(baseTransformations * links[i]->getRestEndPose());
//	}
//}

void Snake::UpdateChainGlobalTransformations(int firstIndex, std::vector<SnakeLink>& chainLinks) {
	/*
			vQ[b] = vQ[p] * dQ[b];
		const Vector3d r = C.row(BE(b,0)).transpose();
		vT[b] = vT[p] - vQ[b]*r + vQ[p]*(r + dT[b]);
*/
// Global transformations of previous link should be computed allready
	for (int i = firstIndex; i < chainLinks.size(); i++) {
		if (i == 0) {
			chainLinks[i].ApplyTransformations(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
		}
		else {
			chainLinks[i].ApplyTransformations(chainLinks[i - 1].GetGlobalRotation(), chainLinks[i - 1].GetGlobalTranslation());
		}
		//baseTransformations *= links[i]->MakeTransd();
		//links[i]->setTip(baseTransformations * links[i]->getRestEndPose());
	}
}
