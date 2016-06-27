#include "ControllerBehavior.h"

using namespace argos;


/* Footbot dimensions */
static const Real INTERWHEEL_DISTANCE        = 0.14f;
static const Real HALF_INTERWHEEL_DISTANCE   = INTERWHEEL_DISTANCE * 0.5f;
static const Real HALF_INTERWHEEL_DISTANCE_IN_CM = HALF_INTERWHEEL_DISTANCE * 100;
static const Real PROXIMITY_SENSOR_RING_RANGE = 0.1f;

/* Simulator parameters */
static const Real TICKS_PER_SEC = 10;

/* Algorithm parameters */
static const Real MAX_VECLOCITY = 5;
static const Real ANGLE_EPSILON = .1;


ControllerBehavior::ControllerBehavior(Sensors s, Actuators a)
    : sensors(s)
    , actuators(a) {
    actuators.leds.Reset();
}

void ControllerBehavior::moveToBegin(const CVector2& beginning) {
    auto rotationAngle = myPositionToPointAngle(beginning);
    move(rotationAngle);
}

void ControllerBehavior::stop() {
    lastControl = lastRotation = CDegrees(0);
    actuators.wheels.SetLinearVelocity(0,0);
}

void ControllerBehavior::move(const CDegrees& rotationAngle) {
    auto controlAngle = getControl(rotationAngle);
    if (controlAngle.GetAbsoluteValue() > ANGLE_EPSILON)
        rotateForAnAngle(controlAngle);
    else {
        actuators.wheels.SetLinearVelocity(MAX_VECLOCITY, MAX_VECLOCITY);
        lastRotation.SetValue(0);
    }
}

CDegrees ControllerBehavior::getControl(const CDegrees& rotationAngle, Real KP, Real KD) const {
    return KP * rotationAngle + KD * (rotationAngle - lastRotation);
}

void ControllerBehavior::rotateForAnAngle(const CDegrees& angle) {
    lastControl = lastRotation = angle;
    auto rotationDirection = getRotationDirection(angle);
    rotationSpeed = ToRadians(angle).GetAbsoluteValue()
                    * HALF_INTERWHEEL_DISTANCE_IN_CM
                    * TICKS_PER_SEC;
    rotate(rotationDirection);
}

ControllerBehavior::Direction ControllerBehavior::getRotationDirection(const CDegrees& obstacleAngle) const {
    return obstacleAngle.GetValue() > 0.0f ? Direction::Right : Direction::Left;
}

void ControllerBehavior::rotate(Direction rotationDirection) {
    if (rotationDirection == Direction::Right)
        actuators.wheels.SetLinearVelocity(rotationSpeed, -(rotationSpeed * 0.8));
    else if (rotationDirection == Direction::Left)
        actuators.wheels.SetLinearVelocity(-(rotationSpeed * 0.8), rotationSpeed);
}

CDegrees ControllerBehavior::myPositionToPointAngle(const CVector2& point) const {
    CVector2 currentPoint;
    sensors.position.GetReading().Position.ProjectOntoXY(currentPoint);
    CDegrees angle = getAngleBetweenPoints(currentPoint, point);
    CDegrees robotsOrientation = getOrientationOnXY();
    return (robotsOrientation - angle).SignedNormalize();
}

CDegrees ControllerBehavior::getAngleBetweenPoints(const CVector2& a, const CVector2& b) const {
    return ToDegrees(ATan2(b.GetY() - a.GetY(), b.GetX() - a.GetX()));
}

CDegrees ControllerBehavior::getOrientationOnXY() const {
    CRadians angleX, angleY, angleZ;
    sensors.position.GetReading().Orientation.ToEulerAngles(angleZ, angleY, angleX);
    return ToDegrees(angleZ);
}
