#pragma once

#include <argos3/core/utility/math/ray3.h>
#include <argos3/core/utility/math/range.h>
#include <list>

class CoverageGrid {
public:
    struct Cell {
        std::list<argos::CRay3> edges;
        bool isCovered = false;
    };

    void initGrid(argos::CRange<argos::CVector3> limits);
    const std::vector<std::vector<Cell>>& getGrid() const;
    Cell& getCell(const argos::CVector3& position);
private:
    const argos::Real cellSizeInMeters = 0.05f;
    const argos::Real gridLiftOnZ = 0.05f;
    std::vector<std::vector<Cell>> grid;

    Cell createCell(argos::Real x, argos::Real y) const;
};


