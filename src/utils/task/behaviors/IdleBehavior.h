#pragma once

#include "utils/task/behaviors/ControllerBehavior.h"

class IdleBehavior : public ControllerBehavior {
public:
    IdleBehavior(Sensors s, Actuators a) : ControllerBehavior(s, a) {}
    void moveToBegin(const argos::CVector2&) override {}
    void prepare() override {}
    void proceed() override {}
};
