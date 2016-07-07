#pragma once

#include "ExplorerBehavior.h"

class RightExplorerBehavior : public ExplorerBehavior {
    const argos::Real frontThreshold = 0.13;
    const argos::Real sideThreshold = 0.1;
public:
    RightExplorerBehavior(Sensors s, Actuators a);
    argos::CVector2 prepare() override;
    bool isReadyToProceed() const override;

protected:
    argos::CDegrees getRotationAngle() const;
    argos::CCI_FootBotProximitySensor::TReadings getRightProximityReadings() const;

private:
    bool hitWall = false;
};


