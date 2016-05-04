#pragma once

#include <argos3/core/utility/math/ray3.h>
#include <argos3/core/utility/math/range.h>
#include <list>

class CoverageGrid {
public:
    const int maxCellConcentration;

    struct Cell {
        std::list<argos::CRay3> edges;
        argos::CVector3 center;
        int concentration;
    };

    CoverageGrid(int maxCellConcentration, argos::Real cellSizeInMeters = 0.05f, argos::Real gridLiftOnZ = 0.01f);
    void initGrid(argos::CRange<argos::CVector3> limits);
    const std::vector<std::vector<Cell>>& getGrid() const;
    Cell& getCell(const argos::CVector3& position);
private:
    const argos::Real cellSizeInMeters;
    const argos::Real gridLiftOnZ;
    std::vector<std::vector<Cell>> grid;

    Cell createCell(argos::Real x, argos::Real y) const;
};
