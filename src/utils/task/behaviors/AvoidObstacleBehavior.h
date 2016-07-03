#pragma once

#include "ControllerBehavior.h"

class AvoidObstacleBehavior : public ControllerBehavior {
public:
    AvoidObstacleBehavior(Sensors s, Actuators a);
    void proceed() override;
    void prepare() override {};
    bool isRoadClear() const;

private:
    const argos::Real minDistanceFromObstacle;
    const argos::CRange<argos::CDegrees> minAngleFromObstacle;

    argos::CVector2 getWeightedProximityReading() const;
};


