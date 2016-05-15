#include "CoverageGrid.h"
#include "assert.h"

using namespace argos;

CoverageGrid::CoverageGrid(int maxCellConcentration, Real cellSizeInMeters, Real gridLiftOnZ)
        : maxCellConcentration(maxCellConcentration)
        , cellSizeInMeters(cellSizeInMeters)
        , gridLiftOnZ(gridLiftOnZ)
{}

void CoverageGrid::initGrid(CRange<CVector3> limits) {
    arenaLimits = limits;
    Real minX = arenaLimits.GetMin().GetX();
    Real maxX = arenaLimits.GetMax().GetX();
    Real minY = arenaLimits.GetMin().GetY();
    Real maxY = arenaLimits.GetMax().GetY();

    for (Real x = minX; x < maxX; x += cellSizeInMeters) {
        grid.emplace_back();
        for (Real y = minY; y < maxY; y += cellSizeInMeters)
            grid.back().push_back(createCell(x, y));
    }
}

CoverageGrid::Cell CoverageGrid::createCell(Real x, Real y) const {
    CVector3 leftUpperPoint(x, y + cellSizeInMeters, gridLiftOnZ);
    CVector3 rightUpperPoint(x + cellSizeInMeters, y + cellSizeInMeters, gridLiftOnZ);
    CVector3 leftLowerPoint(x, y, gridLiftOnZ);
    CVector3 rightLowerPoint(x + cellSizeInMeters, y, gridLiftOnZ);

    Cell cell;
    const auto centerOffset = cellSizeInMeters / 2;
    cell.center = CVector3(x + centerOffset, y + centerOffset, gridLiftOnZ * 2);
    cell.concentration = maxCellConcentration;
    cell.edges.push_back(CRay3(leftUpperPoint, rightUpperPoint));
    cell.edges.push_back(CRay3(rightUpperPoint, rightLowerPoint));
    cell.edges.push_back(CRay3(rightLowerPoint, leftLowerPoint));
    cell.edges.push_back(CRay3(leftLowerPoint, leftUpperPoint));
    return cell;
}

std::vector<std::vector<CoverageGrid::Cell>>& CoverageGrid::getGrid() {
    return grid;
}

const std::vector<std::vector<CoverageGrid::Cell>>& CoverageGrid::getGrid() const {
    return grid;
}

CoverageGrid::CellIndex CoverageGrid::getCellIndex(const CVector3& position) const {
    if (!arenaLimits.WithinMinBoundIncludedMaxBoundIncluded(position)) {
        std::stringstream s;
        s << "Position (" << position << ") is outside area!";
        THROW_ARGOSEXCEPTION(s.str())
    }
    unsigned translation = static_cast<unsigned>(grid.size() / 2);
    unsigned x = static_cast<unsigned>(std::floor(position.GetX() / cellSizeInMeters) + translation);
    unsigned y = static_cast<unsigned>(std::floor(position.GetY() / cellSizeInMeters) + translation);
    if (x == grid.size())
        x--;
    if (y == grid.size())
        y--;
    if (x >= grid.size() || y >= grid.size()) {
        std::stringstream s;
        s << "Bad cell index (" << x << "," << y << ") calculated for (" << position << ")";
        THROW_ARGOSEXCEPTION(s.str())
    }
    return {x, y};
}

CoverageGrid::Cell& CoverageGrid::getCell(const CVector3& position) {
    auto i = getCellIndex(position);
    return grid.at(i.first).at(i.second);
}

const CoverageGrid::Cell& CoverageGrid::getCell(const CVector3& position) const {
    auto i = getCellIndex(position);
    return grid.at(i.first).at(i.second);
}

const CoverageGrid::Meters CoverageGrid::getCellSize() const {
    return cellSizeInMeters;
}
