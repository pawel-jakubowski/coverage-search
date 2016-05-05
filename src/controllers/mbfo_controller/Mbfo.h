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
    CCI_DifferentialSteeringActuator* wheelsEngine;
    CCI_FootBotProximitySensor* proximitySensor;
    CCI_PositioningSensor* positioningSensor;

    Real velocity;
    Real minDistanceFromObstacle;
    CRange<CDegrees> minAngleFromObstacle;

    MbfoLoopFunction& loopFnc;
    const CoverageGrid* coverage;

    enum class Direction {
        left, right
    };

    /* Bacteria behavior */
    CVector3 findNextBestPositionInVoronoiCell(const VoronoiDiagram::Cell &cell) const;

    const CVector2 getCurrentPosition() const;
    const Real calculateDistance(const CVector2& a, const CVector2& b) const;
    const CoverageGrid::Cell& getCoverageCell(CVector2 index) const;
    const VoronoiDiagram::Cell& getVoronoiCell();

    /* Obstacle avoidance */
    CVector2 getWeightedProximityReading();
    bool isRoadClear(const CVector2& obstacleProximity);
    void avoidObstacle(const CVector2& obstacleProximity);
    Direction getRotationDirection(const CDegrees& obstacleAngle);
    void rotate(Direction rotationDirection);
};

}
