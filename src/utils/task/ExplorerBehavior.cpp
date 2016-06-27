#include "ExplorerBehavior.h"

using namespace std;
using namespace argos;

ExplorerBehavior::ExplorerBehavior(Sensors s, Actuators a,
                                   argos::CColor myColor,
                                   argos::CColor fellowColor,
                                   std::function<bool(const argos::CDegrees&)> isDesiredAngle,
                                   argos::Real frontThreshold,
                                   argos::Real frontAngleEpsilon)
    : ControllerBehavior(s, a)
    , myColor(myColor)
    , fellowColor(fellowColor)
    , isDesiredAngle(isDesiredAngle)
    , frontThreshold(frontThreshold)
    , frontAngleEpsilon(frontAngleEpsilon)
{
    sensors.cameras.left.Enable();
    sensors.cameras.right.Enable();
    sensors.cameras.front.Enable();
    sensors.cameras.back.Enable();
    actuators.leds.SetAllColors(myColor);
}

void ExplorerBehavior::proceed() {
    auto fellowAngle = getfellowAngle();
    LOG << "Fellow angle: " << fellowAngle << endl;
    auto rotationAngle = getRotationAngle();
    if (rotationAngle.GetAbsoluteValue() > angleEpsilon) {
        auto angleDiff = getControl(rotationAngle);
        rotateForAnAngle(angleDiff);
    }
    else if (!isCriticalPoint() && isDesiredAngle(getfellowAngle()))
        move(rotationAngle);
    else
        stop();
}

bool ExplorerBehavior::isCriticalPoint() const {
    LOG << "Convex CP: " << boolalpha << isConvexCP() << "\n"
        << "Concave CP: " << boolalpha << isConcaveCP() << endl;
    return isConvexCP() || isConcaveCP();
}

bool ExplorerBehavior::isConvexCP() const {
    return !isFellowVisible();
}

bool ExplorerBehavior::isConcaveCP() const {
    return !isConvexCP() &&
           getAccumulatedVector(getFrontProximityReadings(), frontThreshold).SquareLength() > 0 &&
           getfellowAngle().GetAbsoluteValue() <= frontAngleEpsilon;
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

CDegrees ExplorerBehavior::getfellowAngle() const {
    CDegrees fellowAngle;
    size_t addedAnglesCounter = 0;
    for (auto& blob : sensors.cameras.left.GetReadings().BlobList)
        if (blob->Color == fellowColor) {
            LOG << "Left : " << leftCameraOffset - CDegrees((blob->X / 10)) << endl;
            fellowAngle += leftCameraOffset - CDegrees((blob->X / 10));
            addedAnglesCounter++;
        }
    for (auto& blob : sensors.cameras.front.GetReadings().BlobList)
        if (blob->Color == fellowColor) {
            LOG << "Front : " << frontCameraOffset - CDegrees((blob->X / 10)) << endl;
            fellowAngle += frontCameraOffset - CDegrees((blob->X / 10));
            addedAnglesCounter++;
        }
    for (auto& blob : sensors.cameras.right.GetReadings().BlobList)
        if (blob->Color == fellowColor) {
            LOG << "Right : " << rightCameraOffset - CDegrees((blob->X / 10)) << endl;
            fellowAngle += rightCameraOffset - CDegrees((blob->X / 10));
            addedAnglesCounter++;
        }
    for (auto& blob : sensors.cameras.back.GetReadings().BlobList)
        if (blob->Color == fellowColor) {
            LOG << "Back : " << backCameraOffset - CDegrees((blob->X / 10)) << endl;
            fellowAngle += backCameraOffset - CDegrees((blob->X / 10));
            fellowAngle.SignedNormalize();
            addedAnglesCounter++;
        }
    fellowAngle /= addedAnglesCounter;
    return fellowAngle;
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
