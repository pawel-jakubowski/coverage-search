#pragma once

#include "ExplorerBehavior.h"

class LeftExplorerBehavior : public ExplorerBehavior {
    const argos::Real frontThreshold = 0.13;
    const argos::Real sideThreshold = 0.1;
public:
    LeftExplorerBehavior(Sensors s, Actuators a);
    argos::CVector2 prepare() override;
    bool isForwardConvexCP() const override;
    bool isReadyToProceed() const override;

protected:
    argos::CDegrees getRotationAngle() const;
    argos::CCI_FootBotProximitySensor::TReadings getLeftProximityReadings() const;

private:
    bool hitWall = false;
};
