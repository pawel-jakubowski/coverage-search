#pragma once

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>

#include <random>

namespace argos {

class PsoController : public CCI_Controller {
    static CVector2 bestNeighbourhoodPosition;
    static Real bestNeighbourhoodSolution;
    const int stepsPerIteration = 100;
    const CDegrees acceptableDelta = CDegrees(1.5);
public:
    PsoController();
    virtual ~PsoController() {}

    virtual void Init(TConfigurationNode& configuration);
    virtual void ControlStep();
    virtual void Reset() {}
    virtual void Destroy() {}
private:
    CCI_DifferentialSteeringActuator* wheelsEngine;
    CCI_ProximitySensor* proximitySensor;
    CCI_PositioningSensor* positioningSensor;
    CCI_LightSensor* lightSensor;

    Real rotationSpeed;
    Real maxVelocity;
    CVector2 velocity;
    Real minDistanceFromObstacle;
    Real inertia;
    Real personalWeight;
    Real neighbourhoodWeight;
    CRange<CDegrees> minAngleFromObstacle;
    CVector2 bestPosition;
    Real bestSolution;

    int stepCounter;
    std::default_random_engine generator;

    enum class Direction {
        left, right
    };

    CVector2 getWeightedProximityReading();
    bool isRoadClear(const CVector2& obstacleProximity);
    Direction getRotationDirection(const CDegrees& obstacleAngle);
    void rotate(Direction rotationDirection);
    void move(const CCI_PositioningSensor::SReading& positioningReading);
    void calculateNewVelocity(const CCI_PositioningSensor::SReading& positioningReading);
    void checkIfBetterSolutionThan(double currentUtility, CVector3 currentPosition,
            double& bestSolution, CVector2& bestPosition);
    void updateUtilities(const CCI_PositioningSensor::SReading& positioningReading);
};

}
