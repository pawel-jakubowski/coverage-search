#pragma once

#include "MbfoLoopFunction.h"

class DynamicMbfoLoopFunction : public MbfoLoopFunction {
public:
    DynamicMbfoLoopFunction();
    ~DynamicMbfoLoopFunction() = default;
    virtual void PreStep() override;
    virtual void PostStep() override;
private:
    unsigned long step;
};
