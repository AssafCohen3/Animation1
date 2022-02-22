#include "Snake.h"


Snake::Snake(): 
	velocity(MIN_VELOCITY),
	sea_green(Eigen::RowVector3d(70. / 255., 252. / 255., 167. / 255.)),
	fabCount(0)
{

}

Snake::~Snake() {
	for (int i = 0; i < links.size(); i++) {
		delete links.at(i);
	}
}

void Snake::InitData(igl::opengl::ViewerData& vd) {
	ScaleSnake(vd);
	direction = Eigen::Vector4d(0, 0, 1, 1);
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
	UpdateTips(CalcLinkParentsTransformation(0), 0);
	CalculateWeights();
	//std::cout << weights << std::endl;
	// retrieve parents for forward kinematics
	RefreshFabrikDestination();
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
	if (checkIfTargetReached()) {
		//fixZAxisRotation();
		//RefreshFabrikDestination();
		//scn.SetAnimation();
	}
	//FabrikStep(vd);
	UpdateMovement();
	UpdateSkinning(vd);
	DrawSkeleton(vd, SP, SE);
	//fixZAxisRotation();
}

void Snake::UpdateMovement() {
	Eigen::Matrix3d headRotation = CalcLinkParentsRotation(SNAKE_LINKS_NUM);
	links[0]->translate(headRotation * Eigen::Vector3d(0, 0, -velocity));
	UpdateTips(Eigen::Matrix4d::Identity(), 0);
}

Eigen::Vector4d Snake::GenerateSnakeDestination() {
	Eigen::Quaterniond headRotation = Eigen::Quaterniond(CalcLinkParentsRotation(SNAKE_LINKS_NUM));
	Eigen::Quaterniond rotationMat = Eigen::Quaterniond(Eigen::AngleAxisd((ANGLE_CHANGE_SIZE / 180.0) * igl::PI, direction.head(3)));
	//Eigen::Quaterniond rotationMat = Eigen::Quaterniond(Eigen::AngleAxisd((ANGLE_CHANGE_SIZE / 180.0), headRotation * direction * headRotation.conjugate() *
	//	direction.head(3)).matrix());
	//Eigen::Matrix3d rotationMat = Eigen::AngleAxisd((ANGLE_CHANGE_SIZE / 180.0), direction.head(3)).matrix();
	Eigen::Quaterniond globalDirection = headRotation * rotationMat * headRotation.conjugate();// headRotation* rotationMat;
	Eigen::Vector3d amt = globalDirection * Eigen::Vector3d(0, 0, -3);
	Eigen::Vector4d destinationPos = links[SNAKE_LINKS_NUM - 1]->getTip() + Eigen::Vector4d(amt(0), amt(1), amt(2), 0);
	return destinationPos;
}

bool Snake::checkIfTargetReached() {
	Eigen::Vector4d finalDistanceV = fabrikDestination - links[SNAKE_LINKS_NUM - 1]->getTip();
	double distance = finalDistanceV.norm();
	if (distance <= FABRIK_DELTA) {
		std::cout << "reached fabrik. distance: " << distance << std::endl;
		return true;
	}
	return false;
}

void Snake::RefreshFabrikDestination() {
	direction = Eigen::Vector4d(0, 0, 1, 0);
	fabrikDestination = GenerateSnakeDestination();
}

void Snake::FabrikStep(igl::opengl::ViewerData& vd) {
	Eigen::Vector4d basePos = links[0]->MakeTransd() * links[0]->getRestStartPose();
	Eigen::Vector4d headPos = links[SNAKE_LINKS_NUM - 1]->getTip();
	Eigen::Vector4d destinationPos = fabrikDestination;
	Eigen::Vector4d b = basePos;
	std::map<int, Eigen::Vector4d> new_points;
	new_points[SNAKE_LINKS_NUM + 1] = destinationPos;
	for (int i = SNAKE_LINKS_NUM; i >= 0; i--) {
		Eigen::Vector4d currentTip = i == 0 ? basePos : links[i-1]->getTip();
		Eigen::Vector4d nextTip = new_points[i + 1];
		Eigen::Vector4d Vri = nextTip - currentTip;
		double lambdai = SNAKE_LINK_LENGTH / Vri.norm();
		new_points[i] = (1 - lambdai) * nextTip + lambdai * currentTip;
	}
	//Eigen::Vector4d advancePoint = new_points[0];
	new_points[0] = b;
	for (int i = 0; i <= SNAKE_LINKS_NUM; i++) {
		Eigen::Vector4d currentTip = new_points[i];
		Eigen::Vector4d nextTip = new_points[i + 1];
		Eigen::Vector4d Vri = nextTip - currentTip;
		double lambdai = SNAKE_LINK_LENGTH / Vri.norm();
		new_points[i + 1] = (1 - lambdai) * currentTip + lambdai * nextTip;
	}
	for (int i = 0; i < SNAKE_LINKS_NUM; i++)
	{
		Eigen::Vector4d currentTip = i == 0 ? basePos : links[i - 1]->getTip();
		Eigen::Vector4d nextTip = links[i]->getTip();
		Eigen::Vector4d Ve = nextTip - currentTip;
		Eigen::Vector4d Vt = new_points[i + 1] - new_points[i];
		Eigen::Vector4d Vr;
		Vr << Ve.cross3(Vt).normalized();
		Eigen::MatrixXd currentLinkRotation = CalcLinkParentsTransformation(i);
		Eigen::MatrixXd nextLinkRotation = currentLinkRotation * links[i]->MakeTransd();
		Eigen::Vector4d eulerAxis = nextLinkRotation.inverse() * Vr;
		double dotProduct = Ve.normalized().dot(Vt.normalized());
		if (abs(dotProduct) > 1) {
			dotProduct = dotProduct < 0 ? -1 : 1;
		}
		double angleToRotate = acos(dotProduct);
		if (angleToRotate < ANGLE_THRESHOLD) {
			continue;
		}
		//links[i]->rotate(eulerAxis.head(3), angleToRotate * STEPS_MULTIPLIER);
		Eigen::Quaterniond rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angleToRotate * STEPS_MULTIPLIER, eulerAxis.head(3)));
		links[i]->rotate(rotation);
		UpdateTips(currentLinkRotation, i + 1);
	}
	//links[0]->translate((direction.MakeTransd() * (new_points[0] - basePos)).head(3) * velocity);
	fabCount += 0;
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

void Snake::updateDirection(igl::opengl::ViewerData& vd, int newDirection) {
	/*if (fabCount < FABRIK_MIN_ITERATIONS) {
		std::cout << "did not finished movement." << std::endl;
	}*/
	animationDelta = "ssss";
	switch (newDirection)
	{
	case GLFW_KEY_UP:
		direction = Eigen::Vector4d(1, 0, 0, 0);
		break;
	case GLFW_KEY_DOWN:
		direction = Eigen::Vector4d(-1, 0, 0, 0);
		break;
	case GLFW_KEY_LEFT:
		direction = Eigen::Vector4d(0, -1, 0, 0);
		break;
	case GLFW_KEY_RIGHT:
		direction = Eigen::Vector4d(0, 1, 0, 0);
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

//void Snake::fixZAxisRotation() {
//	for (int i = 0; i < SNAKE_LINKS_NUM; i++)
//	{
//		Eigen::Vector3d ea = links[i]->GetRotation().matrix().eulerAngles(2, 0, 2);
//		links[i]->rotate(Eigen::Quaterniond(Eigen::AngleAxisd(-ea[2], Eigen::Vector3d(0, 0, 1))));
//		if (i < SNAKE_LINKS_NUM - 1) {
//			links[i + 1]->RotateInSystem(Eigen::Vector3d(0, 0, 1), ea[2]);
//		}
//	}
//}

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
	//Eigen::Vector3d destinationPoint = GenerateSnakeDestination().head(3);
	//Eigen::Vector2i destinationEdge = Eigen::Vector2i(skelEdges(skelEdges.rows() - 1, 1), skelPoints.rows());
	//std::cout << destinationPoint << std::endl;
	skelPoints.conservativeResize(skelPoints.rows() + 1, Eigen::NoChange);
	//skelEdges.conservativeResize(skelEdges.rows() + 2, Eigen::NoChange);
	//skelPoints.row(skelPoints.rows() - 2) = destinationPoint;
	//skelEdges.row(skelEdges.rows() - 1) = destinationEdge;
	skelPoints.row(skelPoints.rows() - 1) = fabrikDestination.head(3);
	/*std::cout << " *************** dir **************" << std::endl;
	std::cout << skelPoints << std::endl;
	std::cout << skelEdges << std::endl;*/
	vd.set_edges(skelPoints, skelEdges, sea_green);
	vd.set_points(skelPoints, Eigen::RowVector3d(1, 0, 0));
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