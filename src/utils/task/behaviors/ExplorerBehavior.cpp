#include "ExplorerBehavior.h"
#include <list>
#include <algorithm>


using namespace std;
using namespace argos;

ExplorerBehavior::ExplorerBehavior(Sensors s, Actuators a,
                                   argos::CColor myColor,
                                   argos::CColor fellowColor,
                                   argos::Real frontThreshold,
                                   argos::Real frontAngleEpsilon)
    : ControllerBehavior(s, a)
    , myColor(myColor)
    , fellowColor(fellowColor)
    , frontThreshold(frontThreshold)
    , frontAngleEpsilon(frontAngleEpsilon)
{
    sensors.cameras.left.Enable();
    sensors.cameras.right.Enable();
    sensors.cameras.front.Enable();
    sensors.cameras.back.Enable();
}

void ExplorerBehavior::turnOnLeds() {
    actuators.leds.SetAllColors(myColor);
}

CVector2 ExplorerBehavior::proceed() {
    auto fellowAngle = getFellowAngle();
//    LOG << "Fellow angle: " << fellowAngle << endl;
    auto rotationAngle = getRotationAngle();
    return move(rotationAngle);
}

bool ExplorerBehavior::isCriticalPoint() const {
//    LOG << "Convex CP: " << boolalpha << isConvexCP() << " ("
//        << boolalpha << isForwardConvexCP() << "), "
//        << "Concave CP: " << boolalpha << isConcaveCP() << endl;
    return isConvexCP() || isConcaveCP();
}

bool ExplorerBehavior::isConvexCP() const {
    if (!isFellowVisible())
        return true;
    else if (isForwardConvexCP())
        return true;
    return false;
}

//bool ExplorerBehavior::isForwardConvexCP() const {
//    auto angles = getFellowAngles();
//    CDegrees maxAngleDiff(0);
//    if (angles.size() > 0)
//        for (auto& a : angles)
//            for (auto& o : angles)
//                if (a > o) {
//                    auto angleDiff = (a - o).UnsignedNormalize();
//                    if (angleDiff > maxAngleDiff)
//                        maxAngleDiff = angleDiff;
//                }
//    return maxAngleDiff >= CDegrees(180);
//}

bool ExplorerBehavior::isConcaveCP() const {
    return !isConvexCP() &&
           getAccumulatedVector(getFrontProximityReadings(), frontThreshold).SquareLength() > 0 &&
           getFellowAngle().GetAbsoluteValue() <= frontAngleEpsilon;
}

bool ExplorerBehavior::isFellowVisible() const {
    auto& leftBlobs = sensors.cameras.left.GetReadings().BlobList;
    auto& frontBlobs = sensors.cameras.front.GetReadings().BlobList;
    auto& rightBlobs = sensors.cameras.right.GetReadings().BlobList;
    auto& backBlobs = sensors.cameras.back.GetReadings().BlobList;

    auto allCameraBlobs = leftBlobs;
    allCameraBlobs.insert(allCameraBlobs.end(), frontBlobs.begin(), frontBlobs.end());
    allCameraBlobs.insert(allCameraBlobs.end(), rightBlobs.begin(), rightBlobs.end());
    allCameraBlobs.insert(allCameraBlobs.end(), backBlobs.begin(), backBlobs.end());

    for (auto& blob : allCameraBlobs)
        if (blob->Color == fellowColor)
            return true;
    return false;
}

CDegrees ExplorerBehavior::getFellowAngle() const {
    CDegrees fellowAngle;
    list <CDegrees> detectedFellowAngles = getFellowAngles();
    for_each(detectedFellowAngles.begin(), detectedFellowAngles.end(), [&] (CDegrees& a) { fellowAngle += a; });
    fellowAngle /= detectedFellowAngles.size();
    return fellowAngle;
}

list <CDegrees> ExplorerBehavior::getFellowAngles() const {
    list<CDegrees> detectedFellowAngles;
    for (auto& blob : sensors.cameras.left.GetReadings().BlobList)
        if (blob->Color == fellowColor)
            detectedFellowAngles.push_back(leftCameraOffset - CDegrees((blob->X / 10)));
    for (auto& blob : sensors.cameras.front.GetReadings().BlobList)
        if (blob->Color == fellowColor)
            detectedFellowAngles.push_back(frontCameraOffset - CDegrees((blob->X / 10)));
    for (auto& blob : sensors.cameras.right.GetReadings().BlobList)
        if (blob->Color == fellowColor)
            detectedFellowAngles.push_back(rightCameraOffset - CDegrees((blob->X / 10)));
    for (auto& blob : sensors.cameras.back.GetReadings().BlobList)
        if (blob->Color == fellowColor)
            detectedFellowAngles.push_back((backCameraOffset - CDegrees((blob->X / 10))).SignedNormalize());
    return detectedFellowAngles;
}

CCI_FootBotProximitySensor::TReadings ExplorerBehavior::getFrontProximityReadings() const {
    const auto& readings = sensors.proximity.GetReadings();
    CCI_FootBotProximitySensor::TReadings frontReadings(readings.begin() + 22, readings.end());
    frontReadings.insert(frontReadings.end(), readings.begin(), readings.begin() + 2);
    return frontReadings;
}

CVector2 ExplorerBehavior::getAccumulatedVector(const CCI_FootBotProximitySensor::TReadings& readings, Real threshold) const {
    CVector2 accumulator;
    for (auto& r : readings)
        if (r.Value >= threshold)
            accumulator += CVector2(r.Value, r.Angle);
    return accumulator;
}
