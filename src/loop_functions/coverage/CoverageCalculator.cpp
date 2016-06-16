#include "CoverageCalculator.h"

using namespace argos;

void CoverageCalculator::Init(TConfigurationNode& t_tree) {
    coverage.initGrid(GetSpace().GetArenaLimits());
}

std::vector<CRay3> CoverageCalculator::getGrid() {
    std::vector<CRay3> grid;
    for (auto& row : coverage.getGrid())
        for (auto& cell : row)
            for (auto& edge : cell.edges)
                grid.emplace_back(edge);
    return grid;
}

REGISTER_LOOP_FUNCTIONS(CoverageCalculator, "calculate_coverage")
