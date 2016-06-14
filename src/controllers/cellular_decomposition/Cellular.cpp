#include <argos3/core/utility/logging/argos_log.h>
#include "Cellular.h"

using namespace std;
using namespace argos;


/* Footbot dimensions */
static const Real INTERWHEEL_DISTANCE        = 0.14f;
static const Real HALF_INTERWHEEL_DISTANCE   = INTERWHEEL_DISTANCE * 0.5f;
static const Real HALF_INTERWHEEL_DISTANCE_IN_CM = HALF_INTERWHEEL_DISTANCE * 100;
static const Real PROXIMITY_SENSOR_RING_RANGE = 0.1f;

/* Simulator parameters */
static const Real TICKS_PER_SEC = 10;


Cellular::Cellular() {}

void Cellular::Init(TConfigurationNode& configuration) {
    wheelsEngine = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    proximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    positioningSensor = GetSensor<CCI_PositioningSensor>("positioning");
    lightSensor = GetSensor<CCI_LightSensor>("light");
    rabRx = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");

    GetNodeAttributeOrDefault(configuration, "velocity", velocity, velocity);
    GetNodeAttributeOrDefault(configuration, "min_distance", minDistanceFromObstacle,
                              minDistanceFromObstacle);

    assert(wheelsEngine != nullptr);
    assert(proximitySensor != nullptr);
    assert(positioningSensor != nullptr);
    assert(lightSensor != nullptr);
    assert(rabRx != nullptr);
}

void Cellular::ControlStep() {
//    CDegrees rotationAngle = getRotationAngle();
//    move(rotationAngle);
    moveToPoint({0,0});
}

void Cellular::moveToPoint(const CVector2& point) {
    CVector2 currentPoint;
    positioningSensor->GetReading().Position.ProjectOntoXY(currentPoint);
    CDegrees angle = getAngleBetweenPoints(currentPoint, point);
    CDegrees robotsOrientation = getOrientationOnXY();
    auto rotationAngle = (robotsOrientation - angle).SignedNormalize();
    move(rotationAngle);
}

CDegrees Cellular::getAngleBetweenPoints(const CVector2& a, const CVector2& b) const {
    auto angle = ToDegrees(ATan2(b.GetY() - a.GetY(), b.GetX() - a.GetX()));
    return angle;
}

CDegrees Cellular::getRotationAngle() const {
    using TReadings = CCI_FootBotProximitySensor::TReadings;
    const auto& readings = proximitySensor->GetReadings();

    TReadings leftReadings(readings.begin() + 3, readings.begin() + 9);
    TReadings backReadings(readings.begin() + 9, readings.begin() + 15);
    TReadings rightReadings(readings.begin() + 15, readings.begin() + 21);
    TReadings frontReadings(readings.begin() + 21, readings.end());
    frontReadings.insert(frontReadings.end(), readings.begin(), readings.begin() + 3);

    CDegrees sideAngle;
    CVector2 accumulator;

    if (behavior == Behavior::FollowLeftBoundary) {
        sideAngle = CDegrees(90);
        accumulator = getAccumulatedVector(frontReadings)
            + getAccumulatedVector(leftReadings);
    }
    else if (behavior == Behavior::FollowRightBoundary) {
        sideAngle = CDegrees(-90);
        accumulator = getAccumulatedVector(frontReadings)
            + getAccumulatedVector(rightReadings);
    }

    CDegrees rotationAngle(0);
    if (accumulator.SquareLength() > 0) {
        rotationAngle = (sideAngle - ToDegrees(accumulator.Angle())).SignedNormalize();
    }
    return rotationAngle;
}

CVector2 Cellular::getAccumulatedVector(const CCI_FootBotProximitySensor::TReadings& readings) const {
    CVector2 accumulator;
    for (auto& r : readings)
        accumulator += CVector2(r.Value, r.Angle);
    return accumulator;
}

void Cellular::move(const CDegrees& rotationAngle) {
    Real KP = 0.05;
    Real KD = 0.05;
    auto angleDiff = KP * rotationAngle + KD * (rotationAngle - lastRotation);

    if (angleDiff.GetAbsoluteValue() > angleEpsilon) {
        rotateForAnAngle(angleDiff);
        lastRotation = angleDiff;
    }
    else {
        wheelsEngine->SetLinearVelocity(velocity, velocity);
        lastRotation.SetValue(0);
    }
}

void Cellular::rotateForAnAngle(const CDegrees& angle) {
    auto rotationDirection = getRotationDirection(angle);
    rotationSpeed = ToRadians(angle).GetAbsoluteValue()
                    * HALF_INTERWHEEL_DISTANCE_IN_CM
                    * TICKS_PER_SEC;
    rotate(rotationDirection);
}

CDegrees Cellular::getOrientationOnXY() {
    CRadians angleX, angleY, angleZ;
    positioningSensor->GetReading().Orientation.ToEulerAngles(angleX, angleY, angleZ);
    CDegrees robotsOrientation = ToDegrees(angleX);
    return robotsOrientation;
}

Cellular::Direction Cellular::getRotationDirection(const CDegrees& obstacleAngle) {
    return obstacleAngle.GetValue() > 0.0f ? Direction::Right : Direction::Left;
}

void Cellular::rotate(Direction rotationDirection) {
    if (rotationDirection == Direction::Right)
        wheelsEngine->SetLinearVelocity(rotationSpeed, -rotationSpeed);
    else if (rotationDirection == Direction::Left)
        wheelsEngine->SetLinearVelocity(-rotationSpeed, rotationSpeed);
}

REGISTER_CONTROLLER(Cellular, "cellular_decomposition_controller")
