#pragma once
#include <Eigen/core>
#include "set"
#include "vector"
#include "igl/opengl/ViewerData.h"
#include "igl/AABB.h"
#include "igl/opengl/glfw/Viewer.h"

class SnakeLink {
public:
    SnakeLink();
    //Eigen::Vector4d getTip() { return tip; };
    //void setTip(Eigen::Vector4d newTip) { tip = newTip; };
    //Eigen::Vector4d getRestStartPose() { return restStartPose; };
    //void setRestStartPose(Eigen::Vector4d pose) { restStartPose = pose; };
    //Eigen::Vector4d getRestEndPose() { return restEndPose; };
    //void setRestEndPose(Eigen::Vector4d pose) { restEndPose = pose; };

    //void translate(Eigen::Vector3d t) { transformation.translate(t); };
    //void rotate(Eigen::Quaterniond q) { transformation.rotate(q); };
    //void RotateInSystem(Eigen::Vector3d rotAxis, double angle) { transformation.rotate(Eigen::AngleAxisd(angle, transformation.rotation().transpose() *
    //    rotAxis.normalized()));
    //};
    //Eigen::Quaterniond GetRotation() { return Eigen::Quaterniond(transformation.rotation()); };
    //Eigen::Matrix4d MakeTransd() { 
    //    return transformation.matrix(); 
    //};
    //Eigen::Quaterniond GetRestPose() { return restPos; };
    //void SetRestPose(Eigen::Quaterniond pose) { restPos = pose; };

    Eigen::Vector3d GetInitialStartTipLocation() { return initialStartTipLocation; };
    void SetInitialStartTipLocation(Eigen::Vector3d loc) { initialStartTipLocation = loc; };

    Eigen::Vector3d GetInitialEndTipLocation() { return initialEndTipLocation; };
    void SetInitialEndTipLocation(Eigen::Vector3d loc) { initialEndTipLocation = loc; };

    Eigen::Vector3d GetCurrentStartTipLocation() { return currentStartTipLocation; };
    void SetCurrentStartTipLocation(Eigen::Vector3d loc) { currentStartTipLocation = loc; };

    Eigen::Vector3d GetCurrentEndTipLocation() { return currentEndTipLocation; };
    void SetCurrentEndTipLocation(Eigen::Vector3d loc) { currentEndTipLocation = loc; };

    Eigen::Vector3d GetLocalTranslation() { return localTranslationVector; };
    void SetLocalTranslation(Eigen::Vector3d t) { localTranslationVector = t; };

    Eigen::Vector3d GetGlobalTranslation() { return globalTranslationVector; };
    void SetGlobalTranslation(Eigen::Vector3d t) { globalTranslationVector = t; };

    Eigen::Quaterniond GetLocalRotation() { return localRotation; };
    void SetLocalRotation(Eigen::Quaterniond rot) { localRotation = rot; };

    Eigen::Quaterniond GetGlobalRotation() { return globalRotation; };
    void SetGlobalRotation(Eigen::Quaterniond rot) { globalRotation = rot; };

    Eigen::Quaterniond GetGlobalAnimationStartOrientation() { return globalAnimationStartOrientation; };
    void SetGlobalAnimationStartOrientation(Eigen::Quaterniond rot) { globalAnimationStartOrientation = rot; };

    Eigen::Quaterniond GetGlobalAnimationTargetOrientation() { return globalAnimationTargetOrientation; };
    void GetGlobalAnimationTargetOrientation(Eigen::Quaterniond rot) { globalAnimationTargetOrientation = rot; };

    void ApplyTransformations(Eigen::Quaterniond rot, Eigen::Vector3d t) {
        globalRotation = rot * localRotation;
        globalTranslationVector = t - globalRotation * initialEndTipLocation + rot * (initialEndTipLocation + localTranslationVector);
        currentStartTipLocation = ApplyToPoint(initialStartTipLocation);
        currentEndTipLocation = ApplyToPoint(initialEndTipLocation);
    }

    Eigen::Vector3d ApplyToPoint(Eigen::Quaterniond rot, Eigen::Vector3d t, Eigen::Vector3d p) {
        Eigen::Quaterniond pOrientation = rot * globalRotation;
        return pOrientation * p * pOrientation.conjugate() + rot * globalTranslationVector * rot.conjugate() + t;
    }

    Eigen::Vector3d ApplyToPoint(Eigen::Vector3d p) {
        return globalRotation * p * globalRotation.conjugate() + globalTranslationVector;
    }

    int id;
    int parentId;
    //Eigen::Vector4d tip;
    //Eigen::Vector4d restStartPose;
    //Eigen::Vector4d restEndPose;
    //Eigen::Affine3d transformation;
    Eigen::Vector3d initialStartTipLocation;
    Eigen::Vector3d initialEndTipLocation;
    Eigen::Vector3d currentStartTipLocation;
    Eigen::Vector3d currentEndTipLocation;
    //Eigen::Quaterniond restPos;
    Eigen::Vector3d localTranslationVector;
    Eigen::Vector3d globalTranslationVector;
    Eigen::Quaterniond localRotation;
    Eigen::Quaterniond globalRotation;
    Eigen::Quaterniond globalAnimationStartOrientation;
    Eigen::Quaterniond globalAnimationTargetOrientation;
};