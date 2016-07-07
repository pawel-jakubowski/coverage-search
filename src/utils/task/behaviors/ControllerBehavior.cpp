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

CVector2 ControllerBehavior::moveToBegin(const CVector2& beginning) {
    auto rotationAngle = myPositionToPointAngle(beginning);
    return move(rotationAngle);
}

void ControllerBehavior::stop() {
    lastControl = lastRotation = CDegrees(0);
    actuators.wheels.SetLinearVelocity(0,0);
}

CVector2 ControllerBehavior::getDefaultVelocity() const {
    return CVector2(MAX_VECLOCITY, CRadians(0));
}

CVector2 ControllerBehavior::move(const CDegrees& rotationAngle) {
    auto controlAngle = getControl(rotationAngle);
    if (rotationAngle.GetAbsoluteValue() > ANGLE_EPSILON) {
        rotateForAnAngle(rotationAngle);
        return getDefaultVelocity().Rotate(ToRadians(rotationAngle));
    }
    else {
        actuators.wheels.SetLinearVelocity(MAX_VECLOCITY, MAX_VECLOCITY);
        lastRotation.SetValue(0);
        return getDefaultVelocity();
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
    return obstacleAngle.GetValue() > 0.0f ? Direction::Left : Direction::Right;
}

void ControllerBehavior::rotate(Direction rotationDirection) {
    if (rotationDirection == Direction::Right)
        actuators.wheels.SetLinearVelocity(rotationSpeed, -rotationSpeed);
    else if (rotationDirection == Direction::Left)
        actuators.wheels.SetLinearVelocity(-rotationSpeed, rotationSpeed);
}

CDegrees ControllerBehavior::myPositionToPointAngle(const CVector2& point) const {
    CVector2 currentPoint;
    sensors.position.GetReading().Position.ProjectOntoXY(currentPoint);
    CDegrees angle = getAngleBetweenPoints(currentPoint, point);
    CDegrees robotsOrientation = getOrientationOnXY();
    return (angle - robotsOrientation).SignedNormalize();
}

CDegrees ControllerBehavior::getAngleBetweenPoints(const CVector2& a, const CVector2& b) const {
    return ToDegrees(ATan2(b.GetY() - a.GetY(), b.GetX() - a.GetX()));
}

CDegrees ControllerBehavior::getOrientationOnXY() const {
    CRadians angleX, angleY, angleZ;
    sensors.position.GetReading().Orientation.ToEulerAngles(angleZ, angleY, angleX);
    return ToDegrees(angleZ);
}
