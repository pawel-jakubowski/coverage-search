#pragma once

#include "ExplorerBehavior.h"

class RightExplorerBehavior : public ExplorerBehavior {
public:
    RightExplorerBehavior(Sensors s, Actuators a);
protected:
    argos::CDegrees getRotationAngle() const;
    argos::CCI_FootBotProximitySensor::TReadings getRightProximityReadings() const;
};


