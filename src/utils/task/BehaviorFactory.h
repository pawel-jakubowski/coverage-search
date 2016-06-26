#pragma once

#include "Task.h"
#include "ControllerBehavior.h"

class BehaviorFactory {
public:
    BehaviorFactory(Sensors s, Actuators a) : sensors(s), actuators(a) {}
    std::shared_ptr<ControllerBehavior> create(Task t);
private:
    Sensors sensors;
    Actuators actuators;
};


