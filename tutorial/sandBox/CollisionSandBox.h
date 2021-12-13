#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
#include <tutorial/sandBox/CollisionData.h>

#define INIT_VELOCITY 0.002

class CollisionSandBox : public igl::opengl::glfw::Viewer
{
public:
	typedef igl::opengl::glfw::Viewer super;
	CollisionSandBox();
	~CollisionSandBox();
	virtual bool load_mesh_from_file(const std::string& mesh_file_name) override;
	virtual void WhenTranslate(int mesh_id) override;
	bool FindCollision(CollisionData* m1, CollisionData* m2);
	bool CheckIntersection(Eigen::AlignedBox<double, 3> b1, Eigen::AlignedBox<double, 3> b2, int m1_id, int m2_id);
	CollisionData* collision_data();
	void InitMesh();
	void InitCollisionData(int dataid);
	void Init(const std::string& config);
	void EraseAllIntersectionBoxes();
	void ChangeDirection(int new_direction);
	void ModifyVelocity(int key);
	double doubleVariable;
private:
	// Prepare array-based edge data structures and priority queue

	std::map<int, CollisionData*> collision_data_map;

	void Animate();


};