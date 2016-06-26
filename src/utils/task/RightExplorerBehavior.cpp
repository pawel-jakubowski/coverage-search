#include "RightExplorerBehavior.h"

using namespace argos;

RightExplorerBehavior::RightExplorerBehavior(Sensors s, Actuators a)
    : ExplorerBehavior(s, a, CColor::RED, CColor::GREEN,
                       [](const CDegrees& a) { return a.GetValue() <= 90; })
{}

CDegrees RightExplorerBehavior::getRotationAngle() const {
    Real frontThreshold = 0.13;
    Real sideThreshold = 0.1;

    auto accumulator = getAccumulatedVector(getFrontProximityReadings(), frontThreshold);
    if (accumulator.SquareLength() == 0)
        accumulator = getAccumulatedVector(getRightProximityReadings(), sideThreshold);

    CDegrees rotationAngle = lastRotation;
    if (accumulator.SquareLength() > 0){
        auto sideAngle = CDegrees(-90);
        rotationAngle = (sideAngle - ToDegrees(accumulator.Angle())).SignedNormalize();
    }
    return rotationAngle;
}

CCI_FootBotProximitySensor::TReadings RightExplorerBehavior::getRightProximityReadings() const {
    const auto& readings = sensors.proximity.GetReadings();
    return CCI_FootBotProximitySensor::TReadings(readings.begin() + 16, readings.begin() + 20);
}
