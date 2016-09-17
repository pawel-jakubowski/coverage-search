#include "LeftExplorerBehavior.h"

using namespace std;
using namespace argos;

LeftExplorerBehavior::LeftExplorerBehavior(Sensors s, Actuators a)
    : ExplorerBehavior(s, a, CColor::GREEN, CColor::RED)
{}

CVector2 LeftExplorerBehavior::prepare() {
    ExplorerBehavior::turnOnLeds();
    
    CDegrees desiredOrientation(0);
    if (hitWall)
        desiredOrientation.SetValue(-90);

    auto orientation = getOrientationOnXY();
    auto angleDiff = (desiredOrientation - orientation).SignedNormalize();
    auto controlAngle = getControl(angleDiff);

    if (controlAngle.GetAbsoluteValue() > angleEpsilon) {
        rotateForAnAngle(controlAngle);
//        LOG << "FLEFT rotate\n";
        return getDefaultVelocity().Rotate(ToRadians(controlAngle));
    }
    else if (!hitWall) {
        hitWall = getAccumulatedVector(getFrontProximityReadings(), frontThreshold).SquareLength() > 0;
//        LOG << "FLEFT move\n";
        return move(CDegrees(0));
    }
}

bool LeftExplorerBehavior::isForwardConvexCP() const {
    auto angles = getFellowAngles();
    auto myAngle = getOrientationOnXY();
    auto threshold = 0.1f;
    for (auto& a : angles) {
        auto realAngle = (myAngle + a).SignedNormalize();
//        LOG << "Real fellow angle: " << myAngle.GetValue() << " + " << a.GetValue() << " = " << realAngle.GetValue() << "\n";
        if (realAngle < CDegrees(90) && realAngle > CDegrees(-90)) {
            const auto& readings = sensors.proximity.GetReadings();
            array<Real, 2> proximityValues = {-1, -1};
            auto unsignedAngle = a.UnsignedNormalize();
            for (size_t i = 0; i < readings.size(); i++) {
                if (i+1 < readings.size())
                    if (unsignedAngle > ToDegrees(readings.at(i).Angle).UnsignedNormalize() &&
                        unsignedAngle < ToDegrees(readings.at(i+1).Angle).UnsignedNormalize()) {
//                        LOG << "Range(" << ToDegrees(readings.at(i).Angle).GetValue() << ", "
//                            << ToDegrees(readings.at(i+1).Angle).GetValue() << "), ";
                        proximityValues = {readings.at(i).Value, readings.at(i+1).Value};
                    }
                else if (unsignedAngle > ToDegrees(readings.at(i).Angle).UnsignedNormalize() ||
                         unsignedAngle < ToDegrees(readings.at(0).Angle).UnsignedNormalize()) {
//                        LOG << "Range(" << ToDegrees(readings.at(i).Angle).GetValue() << ", "
//                            << ToDegrees(readings.at(0).Angle).GetValue() << "), ";
                    proximityValues = {readings.at(i).Value, readings.at(0).Value};
                }
            }
            if (proximityValues.at(0) < 0 || proximityValues.at(1) < 0)
                THROW_ARGOSEXCEPTION("Algorithm do not find proper range for " << unsignedAngle);
//            LOG << "(" << proximityValues.at(0) << " + " << proximityValues.at(1) << ") / 2 = "
//                << ((proximityValues.at(0) + proximityValues.at(1)) / 2) << "\n";
            return ((proximityValues.at(0) + proximityValues.at(1)) / 2) > threshold;
        }
    }
    return false;
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

    size_t howManyRaysDetectObstacle = 0;
    for (auto& r : getLeftProximityReadings())
        if (r.Value > 0)
            howManyRaysDetectObstacle++;

    if (howManyRaysDetectObstacle > 1 && accumulator.SquareLength() > 0) {
        auto sideAngle = CDegrees(90);
        return (ToDegrees(accumulator.Angle()) - sideAngle).SignedNormalize();
    }
    else {
        return CDegrees(15);
    }
}

CCI_FootBotProximitySensor::TReadings LeftExplorerBehavior::getLeftProximityReadings() const {
    const auto& readings = sensors.proximity.GetReadings();
    return CCI_FootBotProximitySensor::TReadings(readings.begin() + 4, readings.begin() + 8);
}
