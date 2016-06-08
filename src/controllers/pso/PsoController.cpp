#include <argos3/core/utility/configuration/argos_configuration.h>
#include <assert.h>

#include <iostream>
#include "PsoController.h"

using namespace std;

namespace argos {

CVector2 PsoController::bestNeighbourhoodPosition = CVector2();
Real PsoController::bestNeighbourhoodSolution = Real(0.0f);

PsoController::PsoController() :
        wheelsEngine(nullptr), proximitySensor(nullptr), positioningSensor(nullptr), lightSensor(
                nullptr), rotationSpeed(0.0f), maxVelocity(0.0f), minDistanceFromObstacle(0.1f), inertia(
                0.0f), personalWeight(0.0f), neighbourhoodWeight(0.0f), minAngleFromObstacle(
                CDegrees(-45.0f), CDegrees(45.0f)), bestSolution(0.0f), stepCounter(0) {
}

void PsoController::Init(TConfigurationNode& configuration) {
    wheelsEngine = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    proximitySensor = GetSensor<CCI_ProximitySensor>("proximity");
    positioningSensor = GetSensor<CCI_PositioningSensor>("positioning");
    lightSensor = GetSensor<CCI_LightSensor>("light");

    GetNodeAttributeOrDefault(configuration, "max_velocity", maxVelocity, maxVelocity);
    GetNodeAttributeOrDefault(configuration, "rotate_speed", rotationSpeed, rotationSpeed);
    GetNodeAttributeOrDefault(configuration, "min_distance", minDistanceFromObstacle,
            minDistanceFromObstacle);
    GetNodeAttributeOrDefault(configuration, "inertia", inertia, inertia);
    GetNodeAttributeOrDefault(configuration, "personal_weight", personalWeight, personalWeight);
    GetNodeAttributeOrDefault(configuration, "neighbourhood_weight", neighbourhoodWeight,
            neighbourhoodWeight);

    assert(wheelsEngine != nullptr);
    assert(proximitySensor != nullptr);
    assert(positioningSensor != nullptr);
    assert(lightSensor != nullptr);

    positioningSensor->GetReading().Position.ProjectOntoXY(bestPosition);
    bestNeighbourhoodPosition = bestPosition;
//    std::cout << "Best position: " << bestPosition << std::endl;
//    std::cout << "Best neighboourhood position: " << bestNeighbourhoodPosition << std::endl;

    calculateNewVelocity(positioningSensor->GetReading());
}

void PsoController::ControlStep() {
    CVector2 obstacleProximity = getWeightedProximityReading();
    CCI_PositioningSensor::SReading positioningReading = positioningSensor->GetReading();
    updateUtilities(positioningReading);

    if (stepCounter % stepsPerIteration == 0) {
        calculateNewVelocity(positioningReading);
    } else {
        if (!isRoadClear(obstacleProximity)) {
            auto obstacleAngle = ToDegrees(obstacleProximity.Angle());
            auto rotationDirection = getRotationDirection(obstacleAngle);
            rotate(rotationDirection);
            velocity.Rotate(-obstacleProximity.Angle());
        } else {
            move(positioningReading);
        }
    }

    ++stepCounter;
}

CVector2 PsoController::getWeightedProximityReading() {
    const std::vector<Real>& proximityValues = proximitySensor->GetReadings();
    std::vector<CRadians> proximityAngles(proximityValues.size(),
            CRadians((ARGOS_PI / 8.0f) * 0.5f));
    CRadians sensorSpacing = CRadians::TWO_PI / proximityValues.size();
    for (UInt32 i = 0; i < proximityAngles.size(); ++i) {
        proximityAngles[i] += i * sensorSpacing;
        proximityAngles[i].SignedNormalize();
    }

    CVector2 accumulator;
    for (size_t i = 0; i < proximityValues.size(); ++i) {
        accumulator += CVector2(proximityValues[i], proximityAngles[i]);
    }

    accumulator /= proximityValues.size();
    return accumulator;
}

void PsoController::updateUtilities(const CCI_PositioningSensor::SReading& positioningReading) {
    auto lightReadings = lightSensor->GetReadings();
    double averageReading = 0;
    for (auto& reading : lightReadings)
        averageReading += reading;
    averageReading /= lightReadings.size();
    checkIfBetterSolutionThan(averageReading, positioningReading.Position, bestSolution,
            bestPosition);
    checkIfBetterSolutionThan(averageReading, positioningReading.Position,
            bestNeighbourhoodSolution, bestNeighbourhoodPosition);
//    std::cout << "[" << GetId() << "] best solution: " << bestSolution << std::endl;
//    std::cout << "Best neighbourhood solution: " << bestNeighbourhoodSolution << std::endl;
}

void PsoController::checkIfBetterSolutionThan(double currentUtility, CVector3 currentPosition,
        double& bestSolution, CVector2& bestPosition) {
    if (bestSolution < currentUtility) {
        bestSolution = currentUtility;
        currentPosition.ProjectOntoXY(bestPosition);
    }
}

bool PsoController::isRoadClear(const CVector2& obstacleProximity) {
    CDegrees obstacleAngle(ToDegrees(obstacleProximity.Angle()));
    CDegrees safeAngle(150.0f);
    return (minAngleFromObstacle.WithinMinBoundIncludedMaxBoundIncluded(obstacleAngle)
            && obstacleProximity.Length() < minDistanceFromObstacle)
            || (obstacleAngle < -safeAngle || obstacleAngle > safeAngle);
}

void PsoController::calculateNewVelocity(
        const CCI_PositioningSensor::SReading& positioningReading) {
    CVector2 position;
    positioningReading.Position.ProjectOntoXY(position);

    std::uniform_real_distribution<double> distribution(0.0f, 1.0f);
    velocity = inertia * velocity
            + personalWeight * distribution(generator) * (bestPosition - position)
            + neighbourhoodWeight * distribution(generator)
                    * (bestNeighbourhoodPosition - position);
    if (velocity.Length() > maxVelocity)
        velocity = CVector2(maxVelocity, velocity.Angle());

//    std::cout << "[" << GetId() << "] new velocity (" << velocity.Length() << ", "
//            << ToDegrees(velocity.Angle()) << ")" << std::endl;
}

void PsoController::move(const CCI_PositioningSensor::SReading& positioningReading) {
    CRadians angleX, angleY, angleZ;
    positioningReading.Orientation.ToEulerAngles(angleX, angleY, angleZ);
//    std::cout << "[" << GetId() << "] orientation (" << ToDegrees(angleX) << ", "
//            << ToDegrees(angleY) << ", " << ToDegrees(angleZ) << ")" << std::endl;

    CDegrees angleDifference = ToDegrees((angleX - velocity.Angle()));
//    std::cout << "[" << GetId() << "] angle difference " << angleDifference << std::endl;

    CRange<CDegrees> angleDifferenceToTurnRight(acceptableDelta, acceptableDelta + CDegrees(180));
    CRange<CDegrees> angleDifferenceToTurnLeft(-acceptableDelta - CDegrees(180), -acceptableDelta);
    if (angleDifferenceToTurnRight.WithinMinBoundIncludedMaxBoundIncluded(angleDifference)
            || angleDifference.GetValue() < angleDifferenceToTurnLeft.GetMin().GetValue())
        rotate(Direction::right);
    else if (angleDifference.GetValue() < angleDifferenceToTurnLeft.GetMax().GetValue())
        rotate(Direction::left);
    else
        wheelsEngine->SetLinearVelocity(velocity.Length(), velocity.Length());
}

PsoController::Direction PsoController::getRotationDirection(const CDegrees& obstacleAngle) {
    return obstacleAngle.GetValue() > 0.0f ? Direction::right : Direction::left;
}

void PsoController::rotate(Direction rotationDirection) {
    if (rotationDirection == Direction::right) {
//        std::cout << "[" << GetId() << "] turn right" << std::endl;
        wheelsEngine->SetLinearVelocity(rotationSpeed, -rotationSpeed);
    } else if (rotationDirection == Direction::left) {
//        std::cout << "[" << GetId() << "] turn left" << std::endl;
        wheelsEngine->SetLinearVelocity(-rotationSpeed, rotationSpeed);
    }
}

}

REGISTER_CONTROLLER(PsoController, "pso_controller")
