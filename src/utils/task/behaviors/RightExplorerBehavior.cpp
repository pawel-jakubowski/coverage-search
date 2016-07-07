#include "RightExplorerBehavior.h"

using namespace std;
using namespace argos;

RightExplorerBehavior::RightExplorerBehavior(Sensors s, Actuators a)
    : ExplorerBehavior(s, a, CColor::RED, CColor::GREEN)
{}

CVector2 RightExplorerBehavior::prepare() {
    ExplorerBehavior::turnOnLeds();

    CDegrees desiredOrientation(180);
    if (hitWall)
        desiredOrientation.SetValue(-90);

    auto orientation = getOrientationOnXY();
    auto angleDiff = (desiredOrientation - orientation).SignedNormalize();
    auto controlAngle = getControl(angleDiff);

    if (controlAngle.GetAbsoluteValue() > angleEpsilon) {
        rotateForAnAngle(controlAngle);
        return getDefaultVelocity().Rotate(ToRadians(controlAngle));
    }
    else if (!hitWall) {
        hitWall = getAccumulatedVector(getFrontProximityReadings(), frontThreshold).SquareLength() > 0;
        return move(CDegrees(0));
    }
}

bool RightExplorerBehavior::isReadyToProceed() const {
    if(!hitWall)
        return false;
    return getAccumulatedVector(getRightProximityReadings(), sideThreshold).SquareLength() > 0;
}

CDegrees RightExplorerBehavior::getRotationAngle() const {
    auto accumulator = getAccumulatedVector(getFrontProximityReadings(), frontThreshold);
    if (accumulator.SquareLength() == 0)
        accumulator = getAccumulatedVector(getRightProximityReadings(), sideThreshold);

    CDegrees rotationAngle = lastRotation;
    if (accumulator.SquareLength() > 0){
        auto sideAngle = CDegrees(-90);
        rotationAngle = (ToDegrees(accumulator.Angle()) - sideAngle).SignedNormalize();
    }
    return rotationAngle;
}

CCI_FootBotProximitySensor::TReadings RightExplorerBehavior::getRightProximityReadings() const {
    const auto& readings = sensors.proximity.GetReadings();
    return CCI_FootBotProximitySensor::TReadings(readings.begin() + 16, readings.begin() + 20);
}
