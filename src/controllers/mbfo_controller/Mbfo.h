#pragma once

#include <core/control_interface/ci_controller.h>
#include <core/utility/math/vector2.h>
#include <plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

#include <loop_functions/MbfoLoopFunction.h>


namespace argos {

class Mbfo : public CCI_Controller {
public:
    Mbfo();
    virtual ~Mbfo() {}

    virtual void Init(TConfigurationNode& configuration);
    virtual void ControlStep();
    virtual void Reset() {}
    virtual void Destroy() {}
private:
    enum class Direction {
        left, right
    };

    struct NextDirection {
        int value;
        Real distance;
        CDegrees angle;
    };

    CCI_DifferentialSteeringActuator* wheelsEngine;
    CCI_FootBotProximitySensor* proximitySensor;
    CCI_PositioningSensor* positioningSensor;

    Real velocity;
    Real rotationSpeed;
    Real minDistanceFromObstacle;
    CRange<CDegrees> minAngleFromObstacle;
    MbfoLoopFunction& loopFnc;

    unsigned long step;
    CDegrees desiredDirection;
    const CoverageGrid* coverage;

    /* Bacteria behavior */
    std::vector<NextDirection> findNextBestDirectionsInVoronoiCell(const VoronoiDiagram::Cell &cell) const;

    const Real calculateDistance(const CVector2& a, const CVector2& b) const;
    const CoverageGrid::Cell& getCoverageCell(CVector2 index) const;
    const VoronoiDiagram::Cell& getVoronoiCell();

    /* Obstacle avoidance */
    CVector2 getWeightedProximityReading();
    bool isRoadClear(const CVector2& obstacleProximity);
    void avoidObstacle(const CVector2& obstacleProximity);
    Direction getRotationDirection(const CDegrees& obstacleAngle);
    void rotate(Direction rotationDirection);

    CDegrees getAngleBetweenPoints(const CVector3 &a, const CVector3 &b) const;

    void swim() const;

    void tumble(const CDegrees &robotsOrientation);
};

}
