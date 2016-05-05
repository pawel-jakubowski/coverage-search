#pragma once

#include "CoverageCell.h"
#include <argos3/core/utility/math/range.h>

class CoverageGrid {
public:
    using Cell = CoverageCell;

    const int maxCellConcentration;

    CoverageGrid(int maxCellConcentration, argos::Real cellSizeInMeters = 0.05f, argos::Real gridLiftOnZ = 0.01f);
    void initGrid(argos::CRange<argos::CVector3> limits);
    const std::vector<std::vector<Cell>>& getGrid() const;
    argos::CVector2 getCellIndex(const argos::CVector3& position) const;
    Cell& getCell(const argos::CVector3& position);
    const Cell& getCell(const argos::CVector3& position) const;
private:
    const argos::Real cellSizeInMeters;
    const argos::Real gridLiftOnZ;
    std::vector<std::vector<Cell>> grid;

    Cell createCell(argos::Real x, argos::Real y) const;
};
