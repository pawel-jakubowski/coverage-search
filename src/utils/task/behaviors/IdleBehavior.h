#pragma once

#include "utils/task/behaviors/ControllerBehavior.h"

class IdleBehavior : public ControllerBehavior {
public:
    IdleBehavior(Sensors s, Actuators a) : ControllerBehavior(s, a) {}
    void stop() override {
        actuators.leds.Reset();
        sensors.cameras.left.Disable();
        sensors.cameras.right.Disable();
        sensors.cameras.front.Disable();
        sensors.cameras.back.Disable();
        ControllerBehavior::stop();
    }
    argos::CVector2 prepare() override { return argos::CVector2(); }
    argos::CVector2 proceed() override { return argos::CVector2(); }
};
