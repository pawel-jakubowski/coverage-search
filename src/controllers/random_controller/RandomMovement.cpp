#include "Mbfo.h"

#include <assert.h>

using namespace std;

namespace argos {

RandomMovement::RandomMovement()
    : wheelsEngine(nullptr)
    , proximitySensor(nullptr)
    , velocity(5.0f)
    , minDistanceFromObstacle(0.1f)
    , minAngleFromObstacle(CDegrees(-45.0f), CDegrees(45.0f))
{}

void RandomMovement::Init(TConfigurationNode& configuration) {
    wheelsEngine = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    proximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");

    GetNodeAttributeOrDefault(configuration, "velocity", velocity, velocity);
    GetNodeAttributeOrDefault(configuration, "min_distance", minDistanceFromObstacle,
            minDistanceFromObstacle);

    assert(wheelsEngine != nullptr);
    assert(proximitySensor != nullptr);
}

void RandomMovement::ControlStep() {
    CVector2 obstacleProximity = getWeightedProximityReading();
    if (isRoadClear(obstacleProximity)) {
        wheelsEngine->SetLinearVelocity(velocity, velocity);
    }
    else {
        auto obstacleAngle = ToDegrees(obstacleProximity.Angle());
        auto rotationDirection = getRotationDirection(obstacleAngle);
        rotate(rotationDirection);
    }
}

CVector2 RandomMovement::getWeightedProximityReading() {
    const CCI_FootBotProximitySensor::TReadings& proximityReadings = proximitySensor->GetReadings();
    CVector2 accumulator;
    for (size_t i = 0; i < proximityReadings.size(); ++i) {
        accumulator += CVector2(proximityReadings[i].Value, proximityReadings[i].Angle);
    }
    accumulator /= proximityReadings.size();
    return accumulator;
}

bool RandomMovement::isRoadClear(const CVector2& obstacleProximity) {
    CDegrees obstacleAngle(ToDegrees(obstacleProximity.Angle()));
    CDegrees safeAngle(150.0f);
    return (minAngleFromObstacle.WithinMinBoundIncludedMaxBoundIncluded(obstacleAngle)
            && obstacleProximity.Length() < minDistanceFromObstacle) ||
            (obstacleAngle < -safeAngle || obstacleAngle > safeAngle);
}

RandomMovement::Direction RandomMovement::getRotationDirection(const CDegrees& obstacleAngle) {
    return obstacleAngle.GetValue() > 0.0f ? Direction::right : Direction::left;
}

void RandomMovement::rotate(Direction rotationDirection) {
    if (rotationDirection == Direction::right)
        wheelsEngine->SetLinearVelocity(velocity, -0.1*velocity);
    else if (rotationDirection == Direction::left)
        wheelsEngine->SetLinearVelocity(-0.1*velocity, velocity);
}

}

REGISTER_CONTROLLER(RandomMovement, "random_controller")
