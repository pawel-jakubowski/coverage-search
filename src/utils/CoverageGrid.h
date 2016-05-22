#pragma once

#include "CoverageCell.h"
#include <argos3/core/utility/math/range.h>

class CoverageGrid {
public:
    using Cell = CoverageCell;
    using Meters = argos::Real;
    using CellIndex = std::pair<unsigned, unsigned>;

    const int maxCellConcentration;

    CoverageGrid(int maxCellConcentration, argos::Real cellSizeInMeters = 0.05f, argos::Real gridLiftOnZ = 0.01f);
    void initGrid(argos::CRange<argos::CVector3> limits);
    std::vector<std::vector<Cell>>& getGrid();
    const std::vector<std::vector<Cell>>& getGrid() const;
    CellIndex getCellIndex(const argos::CVector3& position) const;
    Cell& getCell(const argos::CVector3& position);
    const Cell& getCell(const argos::CVector3& position) const;
    const Meters getCellSize() const;

    const double getCoverageValue();

private:
    const Meters cellSizeInMeters;
    const argos::Real gridLiftOnZ;
    int size;
    std::vector<std::vector<Cell>> grid;
    argos::CRange<argos::CVector3> arenaLimits;

    Cell createCell(argos::Real x, argos::Real y) const;
};
