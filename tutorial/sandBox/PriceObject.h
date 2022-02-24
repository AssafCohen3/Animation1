

#pragma once
#include <Eigen/core>
#include "set"
#include "vector"
#include "igl/opengl/ViewerData.h"
#include "igl/AABB.h"
#include "igl/opengl/glfw/Viewer.h"

class PriceObject {
public:
    PriceObject(int id) :
        initialLocation(Eigen::Vector3d::Zero()),
        eaten(true),
        maxTravelDistance(0),
        direction(Eigen::Vector3d::Zero()),
        velocity(0.0),
        id(id)
    {

    }

    void Initialize(igl::opengl::ViewerData& vd) {
        kTree.init(vd.V, vd.F);
        DrawLayout(vd);
    }
    Eigen::Vector3d GetInitialLocation() { return initialLocation; };
    void SetInitialLocation(Eigen::Vector3d loc) { initialLocation = loc; };

    Eigen::Vector3d GetDirection() { return direction; };
    void SetDirection(Eigen::Vector3d dir) { direction = dir; };

    double GetVelocity() { return velocity; };
    void SetVelocity(double newVelocity) { velocity = newVelocity; };

    double GetMaxTravelDistance() { return maxTravelDistance; };
    void SetMaxTravelDistance(double dist) { maxTravelDistance = dist; };

    bool IsEaten() { return eaten; };
    void SetEaten(bool b) { eaten = b; };

    void advance(igl::opengl::ViewerData& vd) {
        vd.MyTranslate(direction * velocity, true);
        Eigen::Vector3d newLocation = vd.MakeTransd().col(3).head(3);
        double dist = (newLocation - initialLocation).norm();
        if (dist > maxTravelDistance) {
            std::cout << "above max. old point: " << initialLocation << ", new point: " << newLocation << ", distance: " << dist << std::endl;
            direction = -direction;
        }
    }

    void ReseedObject(igl::opengl::ViewerData& vd, double x, double y, double z, double velocity, double travelDistance, Eigen::Vector3d direction) {
        this->initialLocation = Eigen::Vector3d(x, y, z);
        vd.MyTranslate(initialLocation, true);
        this->velocity = velocity;
        this->maxTravelDistance = travelDistance;
        this->direction = direction;
        std::cout << "Reseeded " << id << " with location: " << initialLocation << ", velocity: " << velocity << ", max travel: " << maxTravelDistance << ", direction: " << direction << std::endl;
    }

    void DrawLayout(igl::opengl::ViewerData& vd) {
        if (4 == 4) {
            // Skip drawing layout as in game its not very useful
            return;
        }
        Eigen::MatrixXd V_box(8, 3);
        for (int i = 0; i < 8; i++) {
            V_box.row(i) = kTree.m_box.corner(static_cast<Eigen::AlignedBox3d::CornerType>(i));
        }
        // Edges of the bounding box
        Eigen::MatrixXi E_box(12, 2);
        E_box <<
            0, 1,
            1, 3,
            2, 3,
            2, 0,
            4, 5,
            5, 7,
            6, 7,
            6, 4,
            2, 6,
            0, 4,
            1, 5,
            7, 3;
        vd.set_points(V_box, Eigen::RowVector3d(1,0,0));
        vd.set_edges(V_box, E_box, Eigen::RowVector3d(0, 0, 1));
    }

    int GetId() { return id; };
    void SetId(int id) { this->id = id; };

    igl::AABB<Eigen::MatrixXd, 3>& GetKTree() { return kTree; };


    int id;
    Eigen::Vector3d initialLocation;
    Eigen::Vector3d direction;
    double velocity;
    double maxTravelDistance;    
    bool eaten;
    igl::AABB<Eigen::MatrixXd, 3> kTree;
};