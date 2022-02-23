

#pragma once
#include <Eigen/core>
#include "set"
#include "vector"
#include "igl/opengl/ViewerData.h"
#include "igl/AABB.h"
#include "igl/opengl/glfw/Viewer.h"

class PriceObject {
public:
    PriceObject() :
        initialLocation(Eigen::Vector3d::Zero()),
        eaten(false),
        maxTravelDistance(0),
        direction(Eigen::Vector3d::Zero()),
        velocity(0.0),
        id(-1)
    {

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
        Eigen::Vector3d newLocation = vd.MakeTransd() * initialLocation;
        double dist = (newLocation - initialLocation).norm();
        if (dist > maxTravelDistance) {
            direction = -direction;
        }
    }

    int id;
    Eigen::Vector3d initialLocation;
    Eigen::Vector3d direction;
    double velocity;
    double maxTravelDistance;    
    bool eaten;
};