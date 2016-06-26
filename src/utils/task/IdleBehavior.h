#pragma once

#include "ControllerBehavior.h"

class IdleBehavior : public ControllerBehavior {
public:
    IdleBehavior(Sensors s, Actuators a) : ControllerBehavior(s, a) {}
    void moveToBegin(const argos::CVector2&) override {}
    void proceed() override {}
    void stop() override {}
};
