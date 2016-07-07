#pragma once

#include "utils/task/behaviors/ControllerBehavior.h"

class IdleBehavior : public ControllerBehavior {
public:
    IdleBehavior(Sensors s, Actuators a) : ControllerBehavior(s, a) {}
    argos::CVector2 moveToBegin(const argos::CVector2&) override { return argos::CVector2(); }
    argos::CVector2 prepare() override { return argos::CVector2(); }
    argos::CVector2 proceed() override { return argos::CVector2(); }
};
