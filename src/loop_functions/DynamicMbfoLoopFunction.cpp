#include "DynamicMbfoLoopFunction.h"

static const unsigned long CHEMOTAXIS_LENGTH = 10;

DynamicMbfoLoopFunction::DynamicMbfoLoopFunction()
    : step(0)
{}

void DynamicMbfoLoopFunction::PreStep() {
    MbfoLoopFunction::PreStep();
    if (step % CHEMOTAXIS_LENGTH == 0)
        update();
}

void DynamicMbfoLoopFunction::PostStep() {
    MbfoLoopFunction::PostStep();
    step++;
}

REGISTER_LOOP_FUNCTIONS(DynamicMbfoLoopFunction, "dynamic_mbfo_loop_fcn")
