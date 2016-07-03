#include "AvoidObstacleBehavior.h"

using namespace std;
using namespace argos;

/* Algorithm parameters */
static const Real MAX_VECLOCITY = 5;
static const Real ANGLE_EPSILON = .1;

AvoidObstacleBehavior::AvoidObstacleBehavior(Sensors s, Actuators a)
    : ControllerBehavior(s, a)
    , minDistanceFromObstacle(0.035f)
    , minAngleFromObstacle(CDegrees(-75.0f), CDegrees(75.0f))
{}

void AvoidObstacleBehavior::proceed() {
    LOG << "Avoid obstacle!" << endl;
    auto obstacleAngle = ToDegrees(getWeightedProximityReading().Angle());
    LOG << "Obstacle angle " << obstacleAngle << endl;
    auto controlAngle = getControl(obstacleAngle);
    move(controlAngle);
}

bool AvoidObstacleBehavior::isRoadClear() const {
    auto obstacleProximity = getWeightedProximityReading();
    CDegrees obstacleAngle(ToDegrees(obstacleProximity.Angle()));
    LOG << "Detected obstacle at angle " << obstacleAngle << endl;
    LOG << "Proximity len " << obstacleProximity.Length() << endl;
    CDegrees safeAngle(150.0f);
    if (obstacleProximity.Length() > minDistanceFromObstacle)
        if (minAngleFromObstacle.WithinMinBoundIncludedMaxBoundIncluded(obstacleAngle))
            return false;
        else if (obstacleAngle < -safeAngle || obstacleAngle > safeAngle)
            return false;
    return true;
}

CVector2 AvoidObstacleBehavior::getWeightedProximityReading() const {
    const auto& readings = sensors.proximity.GetReadings();

    CVector2 accumulator;
    for (auto r : readings)
        accumulator += CVector2(r.Value, r.Angle);

    accumulator /= readings.size();
    return accumulator;
}