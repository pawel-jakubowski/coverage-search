#pragma once

#include "ControllerBehavior.h"

class SweeperBehavior : public ControllerBehavior {
public:
    using ControllerBehavior::ControllerBehavior;
    void proceed() override {}
};


