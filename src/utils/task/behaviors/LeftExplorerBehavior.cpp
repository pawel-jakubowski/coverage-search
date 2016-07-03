#include "LeftExplorerBehavior.h"

using namespace std;
using namespace argos;

LeftExplorerBehavior::LeftExplorerBehavior(Sensors s, Actuators a)
    : ExplorerBehavior(s, a, CColor::GREEN, CColor::RED)
{}

void LeftExplorerBehavior::prepare() {
    ExplorerBehavior::prepare();
    
    CDegrees desiredOrientation(0);
    if (hitWall)
        desiredOrientation.SetValue(-90);

    auto orientation = getOrientationOnXY();
    auto angleDiff = (orientation - desiredOrientation).SignedNormalize();
    auto controlAngle = getControl(angleDiff);

    if (controlAngle.GetAbsoluteValue() > angleEpsilon)
        rotateForAnAngle(controlAngle);
    else if (!hitWall) {
        hitWall = getAccumulatedVector(getFrontProximityReadings(), frontThreshold).SquareLength() > 0;
        move(CDegrees(0));
    }
}

bool LeftExplorerBehavior::isReadyToProceed() const {
    if(!hitWall)
        return false;
    return getAccumulatedVector(getLeftProximityReadings(), sideThreshold).SquareLength() > 0;
}

CDegrees LeftExplorerBehavior::getRotationAngle() const {
    auto accumulator = getAccumulatedVector(getFrontProximityReadings(), frontThreshold);
    if (accumulator.SquareLength() == 0)
        accumulator = getAccumulatedVector(getLeftProximityReadings(), sideThreshold);

    CDegrees rotationAngle = lastRotation;
    if (accumulator.SquareLength() > 0) {
        auto sideAngle = CDegrees(90);
        rotationAngle = (sideAngle - ToDegrees(accumulator.Angle())).SignedNormalize();
    }
    return rotationAngle;
}

CCI_FootBotProximitySensor::TReadings LeftExplorerBehavior::getLeftProximityReadings() const {
    const auto& readings = sensors.proximity.GetReadings();
    return CCI_FootBotProximitySensor::TReadings(readings.begin() + 4, readings.begin() + 8);
}