#pragma once

#include <core/control_interface/ci_controller.h>
#include <core/utility/math/vector2.h>
#include <plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <loop_functions/cellular_decomposition/CellularDecomposition.h>
#include <utils/Task.h>


namespace argos {

class Cellular : public CCI_Controller {
    const Real angleEpsilon = 1;
public:
    Cellular();
    virtual ~Cellular() {}

    virtual void Init(TConfigurationNode& configuration);
    virtual void ControlStep();
    virtual void Destroy() {}

private:
    enum class Direction { Left, Right };

    CCI_DifferentialSteeringActuator* wheelsEngine = nullptr;
    CCI_FootBotProximitySensor* proximitySensor = nullptr;
    CCI_PositioningSensor* positioningSensor = nullptr;
    CCI_LightSensor* lightSensor = nullptr;
    CCI_RangeAndBearingSensor* rabRx = nullptr;

    Real velocity;
    Real rotationSpeed;
    Real minDistanceFromObstacle;
    CRange<CDegrees> minAngleFromObstacle;
    CDegrees lastRotation;

    Real distanceFromWall;
    Task currentTask;

    CellularDecomposition& loopFnc;

    void rotateForAnAngle(const CDegrees &angle);
    CDegrees getOrientationOnXY();
    Direction getRotationDirection(const CDegrees& obstacleAngle);
    void rotate(Direction rotationDirection);
    void move(const CDegrees& rotationAngle);
    CVector2 getAccumulatedVector(const CCI_FootBotProximitySensor::TReadings& readings) const;
    CDegrees getRotationAngle() const;
    CDegrees getAngleBetweenPoints(const CVector2& a, const CVector2& b) const;
    void moveToPoint(const CVector2& point);
};

}
