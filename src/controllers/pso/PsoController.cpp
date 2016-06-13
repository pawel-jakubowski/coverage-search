#include <argos3/core/utility/configuration/argos_configuration.h>
#include <assert.h>

#include <iostream>
#include "PsoController.h"

using namespace std;

namespace argos {

CVector2 PsoController::bestNeighbourhoodPosition = CVector2();
Real PsoController::bestNeighbourhoodSolution = Real(0.0f);

PsoController::PsoController()
    : loopFnc(dynamic_cast<ClosestDistance&>(CSimulator::GetInstance().GetLoopFunctions()))
    , rotationSpeed(0.0f)
    , maxVelocity(0.0f)
    , minDistanceFromObstacle(0.1f)
    , inertia(0.0f)
    , personalWeight(0.0f)
    , neighbourhoodWeight(0.0f)
    , minAngleFromObstacle(CDegrees(-45.0f), CDegrees(45.0f))
    , bestSolution(0.0f)
    , stepCounter(0)
{}

void PsoController::Init(TConfigurationNode& configuration) {
    wheelsEngine = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    proximitySensor = GetSensor<CCI_ProximitySensor>("proximity");
    positioningSensor = GetSensor<CCI_PositioningSensor>("positioning");
    lightSensor = GetSensor<CCI_LightSensor>("light");
    rabRx = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");

    std::string targetTypeStr;
    GetNodeAttribute(configuration, "target", targetTypeStr);
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
    assert(rabRx != nullptr);

    if (targetTypeStr == "light")
        targetType = TargetType::Light;
    else if (targetTypeStr == "robot")
        targetType = TargetType::Robot;
    else
        THROW_ARGOSEXCEPTION("Unknown target type: " + targetTypeStr);

    positioningSensor->GetReading().Position.ProjectOntoXY(bestPosition);
    bestNeighbourhoodPosition = bestPosition;

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
    double utilityValue = getCurrentUtilityValue();
    checkIfBetterSolutionThan(utilityValue, positioningReading.Position, bestSolution,
            bestPosition);
    checkIfBetterSolutionThan(utilityValue, positioningReading.Position,
            bestNeighbourhoodSolution, bestNeighbourhoodPosition);
}

double PsoController::getCurrentUtilityValue() {
    if (targetType == TargetType::Light) {
        loopFnc.addRobotPosition(positioningSensor->GetReading().Position);
        return getAverageLightValue();
    }
    else if (targetType == TargetType::Robot) {
        detectTargets();
        return 0;
    }
    THROW_ARGOSEXCEPTION("PSO cannot calculate utility value!");
}

void PsoController::detectTargets() {
    auto position = positioningSensor->GetReading().Position;
    auto packets = rabRx->GetReadings();
    for(auto& packet : packets) {
        int id;
        Real posX, posY, posZ;
        packet.Data >> id >> posX >> posY >> posZ;
        if (id == 0) continue; // Skip default message send by pso robots
        CVector3 targetPosition(posX, posY, posZ);
        loopFnc.addTargetPosition(id, targetPosition);
    }
}

double PsoController::getAverageLightValue() const {
    auto lightReadings = lightSensor->GetReadings();
    double averageReading = 0;
    for (auto& reading : lightReadings)
        averageReading += reading;
    averageReading /= lightReadings.size();
    return averageReading;
}

void PsoController::checkIfBetterSolutionThan(double currentUtility, CVector3 currentPosition,
        double& bestSolution, CVector2& bestPosition) {
    if (bestSolution <= currentUtility) {
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

}

void PsoController::move(const CCI_PositioningSensor::SReading& positioningReading) {
    CRadians angleX, angleY, angleZ;
    positioningReading.Orientation.ToEulerAngles(angleX, angleY, angleZ);

    CDegrees angleDifference = ToDegrees((angleX - velocity.Angle()));

    CRange<CDegrees> angleDifferenceToTurnRight(acceptableDelta, acceptableDelta + CDegrees(180));
    CRange<CDegrees> angleDifferenceToTurnLeft(-acceptableDelta - CDegrees(180), -acceptableDelta);
    if (angleDifferenceToTurnRight.WithinMinBoundIncludedMaxBoundIncluded(angleDifference)
            || angleDifference.GetValue() < angleDifferenceToTurnLeft.GetMin().GetValue())
        rotate(Direction::Right);
    else if (angleDifference.GetValue() < angleDifferenceToTurnLeft.GetMax().GetValue())
        rotate(Direction::Left);
    else
        wheelsEngine->SetLinearVelocity(velocity.Length(), velocity.Length());
}

PsoController::Direction PsoController::getRotationDirection(const CDegrees& obstacleAngle) {
    return obstacleAngle.GetValue() > 0.0f ? Direction::Right : Direction::Left;
}

void PsoController::rotate(Direction rotationDirection) {
    if (rotationDirection == Direction::Right) {
        wheelsEngine->SetLinearVelocity(rotationSpeed, -rotationSpeed);
    } else if (rotationDirection == Direction::Left) {
        wheelsEngine->SetLinearVelocity(-rotationSpeed, rotationSpeed);
    }
}

}

REGISTER_CONTROLLER(PsoController, "pso_controller")
