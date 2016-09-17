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

bool RightExplorerBehavior::isForwardConvexCP() const {
    auto angles = getFellowAngles();
    auto myAngle = getOrientationOnXY();
    auto threshold = 0.1f;
    for (auto& a : angles) {
        auto realAngle = (myAngle + a).SignedNormalize();
//        LOG << "Real fellow angle: " << myAngle.GetValue() << " + " << a.GetValue() << " = " << realAngle.GetValue() << "\n";
        if (realAngle > CDegrees(90) || realAngle < CDegrees(-90)) {
            const auto& readings = sensors.proximity.GetReadings();
            array<Real, 2> proximityValues = {-1, -1};
            auto unsignedAngle = a.UnsignedNormalize();
            for (size_t i = 0; i < readings.size(); i++) {
                if (i+1 < readings.size() &&
                    unsignedAngle > ToDegrees(readings.at(i).Angle).UnsignedNormalize() &&
                    unsignedAngle < ToDegrees(readings.at(i+1).Angle).UnsignedNormalize()) {
                    proximityValues = {readings.at(i).Value, readings.at(i+1).Value};
                }
                else if (unsignedAngle > ToDegrees(readings.at(i).Angle).UnsignedNormalize() ||
                         unsignedAngle < ToDegrees(readings.at(0).Angle).UnsignedNormalize()) {
                    proximityValues = {readings.at(i).Value, readings.at(0).Value};
                }
            }
            if (proximityValues.at(0) < 0 || proximityValues.at(1) < 0)
                THROW_ARGOSEXCEPTION("Algorithm do not find proper range for " << unsignedAngle);
            return ((proximityValues.at(0) + proximityValues.at(1)) / 2) > threshold;
        }
    }
    return false;
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

    size_t howManyRaysDetectObstacle = 0;
    for (auto& r : getRightProximityReadings())
        if (r.Value > 0)
            howManyRaysDetectObstacle++;

    if (howManyRaysDetectObstacle > 1 && accumulator.SquareLength() > 0) {
        auto sideAngle = CDegrees(-90);
        return (ToDegrees(accumulator.Angle()) - sideAngle).SignedNormalize();
    }
    else {
        return CDegrees(-15);
    }
}

CCI_FootBotProximitySensor::TReadings RightExplorerBehavior::getRightProximityReadings() const {
    const auto& readings = sensors.proximity.GetReadings();
    return CCI_FootBotProximitySensor::TReadings(readings.begin() + 16, readings.begin() + 20);
}
