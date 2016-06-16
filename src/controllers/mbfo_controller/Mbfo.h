#pragma once

#include <core/control_interface/ci_controller.h>
#include <core/utility/math/vector2.h>
#include <plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <loop_functions/mbfo/MbfoLoopFunction.h>


namespace argos {

class Mbfo : public CCI_Controller {
public:
    Mbfo();
    virtual ~Mbfo() {}

    virtual void Init(TConfigurationNode& configuration);
    virtual void Reset();
    virtual void ControlStep();
    virtual void Destroy() {}

private:
    enum class Direction {
        left, right
    };

    struct NextDirection {
        int value;
        Real distance;
        CDegrees angle;
        CVector2 index;
    };

    const Real angleEpsilon = 1;

    CCI_DifferentialSteeringActuator* wheelsEngine = nullptr;
    CCI_FootBotProximitySensor* proximitySensor = nullptr;
    CCI_PositioningSensor* positioningSensor = nullptr;
    CCI_LightSensor* lightSensor = nullptr;
    CCI_RangeAndBearingSensor* rabRx = nullptr;

    bool stopped;
    Real velocity;
    Real rotationSpeed;
    Real minDistanceFromObstacle;
    CRange<CDegrees> minAngleFromObstacle;
    MbfoLoopFunction& loopFnc;

    std::string currentCellId;
    unsigned long step;
    CDegrees desiredDirection;
    const CoverageGrid* coverage = nullptr;

    CVector3 targetsPosition;

    template<class Vector>
    const Real calculateDistance(const Vector& a, const Vector& b) const {
        return (a - b).SquareLength();
    }
    CDegrees getOrientationOnXY();
    bool isCellDone(const VoronoiDiagram::Cell& cell) const;
    const CoverageGrid::Cell& getCoverageCell(CVector2 index) const;
    const VoronoiDiagram::Cell& getVoronoiCell(std::string cellId);
    CDegrees getAngleBetweenPoints(const CVector3 &a, const CVector3 &b) const;

    /* Obstacle avoidance */
    CVector2 getWeightedProximityReading();
    bool isRoadClear(const CVector2& obstacleProximity);
    void avoidObstacle(const CVector2& obstacleProximity);
    Direction getRotationDirection(const CDegrees& obstacleAngle);
    void rotate(Direction rotationDirection);

    /* Bacteria behavior */
    void swim() const;
    void tumble(const CDegrees &robotsOrientation);
    void determineNewDirection();
    CVector3 calculateRobotsInteractionForce() const;
    std::vector<NextDirection> findNextBestDirectionsInVoronoiCell(const VoronoiDiagram::Cell &cell) const;
    void chooseBestDirectionFromVector(std::vector<Mbfo::NextDirection>& nextBestDirections);

    void detectTarget() const;
};

}
