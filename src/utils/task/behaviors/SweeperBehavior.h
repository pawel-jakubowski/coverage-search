#pragma once

#include "ControllerBehavior.h"

class SweeperBehavior : public ControllerBehavior {
public:
    using ControllerBehavior::ControllerBehavior;
    argos::CVector2 prepare() override { return argos::CVector2(); }
    argos::CVector2 proceed() override { return argos::CVector2(); }
};


