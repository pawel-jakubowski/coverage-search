#pragma once

#include <core/control_interface/ci_controller.h>
#include <core/utility/math/vector2.h>
#include <plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

#include <random>

namespace argos {

class Mbfo : public CCI_Controller {
public:
    Mbfo();
    virtual ~Mbfo() {}

    virtual void Init(TConfigurationNode& configuration);
    virtual void ControlStep();
    virtual void Reset() {}
    virtual void Destroy() {}
private:
    CCI_DifferentialSteeringActuator* wheelsEngine;
    CCI_FootBotProximitySensor* proximitySensor;
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
