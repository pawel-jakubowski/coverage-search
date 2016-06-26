#include "LeftExplorerBehavior.h"

using namespace argos;

LeftExplorerBehavior::LeftExplorerBehavior(Sensors s, Actuators a)
    : ExplorerBehavior(s, a, CColor::GREEN, CColor::RED,
                       [](const CDegrees& a) { return a.GetValue() >= -90; })
{}

CDegrees LeftExplorerBehavior::getRotationAngle() const {
    Real frontThreshold = 0.13;
    Real sideThreshold = 0.1;

    auto accumulator = getAccumulatedVector(getFrontProximityReadings(), frontThreshold);
    if (accumulator.SquareLength() == 0)
        accumulator = getAccumulatedVector(getLeftProximityReadings(), sideThreshold);

    CDegrees rotationAngle = lastRotation;
    if (accumulator.SquareLength() > 0){
        auto sideAngle = CDegrees(90);
        rotationAngle = (sideAngle - ToDegrees(accumulator.Angle())).SignedNormalize();
    }
    return rotationAngle;
}

CCI_FootBotProximitySensor::TReadings LeftExplorerBehavior::getLeftProximityReadings() const {
    const auto& readings = sensors.proximity.GetReadings();
    return CCI_FootBotProximitySensor::TReadings(readings.begin() + 4, readings.begin() + 8);
}
