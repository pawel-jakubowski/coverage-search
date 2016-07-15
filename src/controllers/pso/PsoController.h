#pragma once

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>

#include <loop_functions/pso/ClosestDistance.h>
#include <random>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>


namespace argos {

class PsoController : public CCI_Controller {
    static CVector2 bestNeighbourhoodPosition;
    static Real bestNeighbourhoodSolution;
    const int stepsPerIteration = 10;
    const CDegrees acceptableDelta = CDegrees(1.5);
public:
    PsoController();
    virtual ~PsoController() {}

    virtual void Init(TConfigurationNode& configuration);
    virtual void ControlStep();
    virtual void Reset() {}
    virtual void Destroy() {}
private:
    enum class Direction { Left, Right };
    enum class TargetType { Unknown, Light, Robot };

    CCI_DifferentialSteeringActuator* wheelsEngine = nullptr;
    CCI_FootBotProximitySensor* proximitySensor = nullptr;
    CCI_PositioningSensor* positioningSensor = nullptr;
    CCI_LightSensor* lightSensor = nullptr;
    CCI_RangeAndBearingSensor* rabRx = nullptr;

    ClosestDistance& loopFnc;

    TargetType targetType = TargetType::Unknown;
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

    CVector2 getWeightedProximityReading();
    bool isRoadClear(const CVector2& obstacleProximity);
    Direction getRotationDirection(const CDegrees& obstacleAngle);
    void rotate(Direction rotationDirection);
    void move(const CCI_PositioningSensor::SReading& positioningReading);
    void calculateNewVelocity(const CCI_PositioningSensor::SReading& positioningReading);
    void checkIfBetterSolutionThan(double currentUtility, CVector3 currentPosition,
            double& bestSolution, CVector2& bestPosition);
    void updateUtilities(const CCI_PositioningSensor::SReading& positioningReading);

    double getCurrentUtilityValue();
    double getAverageLightValue() const;
    void detectTargets();
};

}
