#include <argos3/core/simulator/simulator.h>
#include "CoverageGridDrawer.h"

using namespace argos;

CoverageGridDrawer::CoverageGridDrawer()
        : coverage(dynamic_cast<CoverageCalculator&>(CSimulator::GetInstance().GetLoopFunctions())) {}

void CoverageGridDrawer::DrawInWorld() {
    auto grid = coverage.getGrid();
    for(const auto& edge : grid)
        DrawRay(edge, CColor::BLACK);
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CoverageGridDrawer, "draw_coverage")
