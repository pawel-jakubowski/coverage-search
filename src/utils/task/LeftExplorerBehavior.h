#pragma once

#include "ExplorerBehavior.h"

class LeftExplorerBehavior : public ExplorerBehavior {
public:
    LeftExplorerBehavior(Sensors s, Actuators a);

protected:
    argos::CDegrees getRotationAngle() const;
    argos::CCI_FootBotProximitySensor::TReadings getLeftProximityReadings() const;
};
