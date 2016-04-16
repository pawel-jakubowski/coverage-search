#include "CoverageCalculator.h"

using namespace argos;

void CoverageCalculator::Init(TConfigurationNode& t_tree) {
    Real minX = GetSpace().GetArenaLimits().GetMin().GetX();
    Real maxX = GetSpace().GetArenaLimits().GetMax().GetX();
    Real minY = GetSpace().GetArenaLimits().GetMin().GetY();
    Real maxY = GetSpace().GetArenaLimits().GetMax().GetY();

    for (Real x = minX; x < maxX; x += cellSizeInMeters)
        grid.emplace_back(CVector3(x, minY, gridLiftOnZ),
                          CVector3(x, maxY, gridLiftOnZ));

    for (Real y = minY; y < maxY; y += cellSizeInMeters)
        grid.emplace_back(CVector3(minX, y, gridLiftOnZ),
                          CVector3(maxX, y, gridLiftOnZ));
}

std::vector<CRay3> CoverageCalculator::getGrid() {
    return grid;
}

REGISTER_LOOP_FUNCTIONS(CoverageCalculator, "calculate_coverage")
