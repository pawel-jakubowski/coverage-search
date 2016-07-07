#pragma once

#include "ControllerBehavior.h"

class AvoidObstacleBehavior : public ControllerBehavior {
public:
    AvoidObstacleBehavior(Sensors s, Actuators a);
    argos::CVector2 proceed() override;
    argos::CVector2 prepare() override { return getDefaultVelocity(); };
    bool isRoadClear(argos::CVector2 desiredVelocity);

private:
    const argos::CDegrees histogramAlpha;
    std::vector<bool> obstacleHistogram;
    const argos::CRange<argos::Real> histogramThresholdHysteresis;

    double getObstacleDistanceFromFootbotProximityReading(argos::Real reading) const;
    std::function<bool(const argos::CRadians&)> generateIsInBoundaryCheck(const argos::CRadians& lowerBoundary,
                                                                          const argos::CRadians& upperBoundary) const;
    void updateObstacleHistogram();
};


