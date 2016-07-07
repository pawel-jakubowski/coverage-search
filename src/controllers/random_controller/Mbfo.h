#pragma once

#include <core/control_interface/ci_controller.h>
#include <core/utility/math/vector2.h>
#include <plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include <random>

namespace argos {

class RandomMovement : public CCI_Controller {
public:
    RandomMovement();
    virtual ~RandomMovement() {}

    virtual void Init(TConfigurationNode& configuration);
    virtual void ControlStep();
    virtual void Reset() {}
    virtual void Destroy() {}
private:
    CCI_DifferentialSteeringActuator* wheelsEngine;
    CCI_FootBotProximitySensor* proximitySensor;
    CCI_PositioningSensor* positioningSensor;
    Real velocity;
    Real minDistanceFromObstacle;
    CRange<CDegrees> minAngleFromObstacle;

    enum class Direction {
        left, right
    };

    CVector2 getWeightedProximityReading();
    bool isRoadClear(const CVector2& obstacleProximity);
    Direction getRotationDirection(const CDegrees& obstacleAngle);
    void rotate(Direction rotationDirection);
};

}
