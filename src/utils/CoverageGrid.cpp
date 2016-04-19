#include "CoverageGrid.h"

using namespace argos;

void CoverageGrid::initGrid(CRange<CVector3> limits) {
    Real minX = limits.GetMin().GetX();
    Real maxX = limits.GetMax().GetX();
    Real minY = limits.GetMin().GetY();
    Real maxY = limits.GetMax().GetY();

    unsigned horizontalSize = static_cast<unsigned>((maxX - minX)/cellSizeInMeters) + 1;
    unsigned verticalSize = static_cast<unsigned>((maxY - minY)/cellSizeInMeters) + 1;

    Real x, y;
    grid.resize(verticalSize);
    for (unsigned i = 0; i < verticalSize; i++) {
        grid.at(i).resize(horizontalSize);
        x = minX + i*cellSizeInMeters;
        if (x > maxX)
            x = maxX;
        for (unsigned j = 0; j < horizontalSize; j++) {
            y = minY + j*cellSizeInMeters;
            if (y > maxY)
                y = maxY;

            grid.at(i).at(j) = createCell(x, y);
        }
    }
}

CoverageGrid::Cell CoverageGrid::createCell(Real x, Real y) const {
    CVector3 leftUpperPoint(x, y + cellSizeInMeters, gridLiftOnZ);
    CVector3 rightUpperPoint(x + cellSizeInMeters, y + cellSizeInMeters, gridLiftOnZ);
    CVector3 leftLowerPoint(x, y, gridLiftOnZ);
    CVector3 rightLowerPoint(x + cellSizeInMeters, y, gridLiftOnZ);

    Cell cell;
    cell.edges.push_back(CRay3(leftUpperPoint, rightUpperPoint));
    cell.edges.push_back(CRay3(rightUpperPoint, rightLowerPoint));
    cell.edges.push_back(CRay3(rightLowerPoint, leftLowerPoint));
    cell.edges.push_back(CRay3(leftLowerPoint, leftUpperPoint));
    return cell;
}

const std::vector<std::vector<CoverageGrid::Cell>>& CoverageGrid::getGrid() const {
    return grid;
}

CoverageGrid::Cell& CoverageGrid::getCell(const CVector3& position) {
    int translation = (grid.size() / 2);
    int x = static_cast<int>(std::round(position.GetX() / cellSizeInMeters)) + translation;
    int y = static_cast<int>(std::round(position.GetY() / cellSizeInMeters)) + translation;
    return grid.at(x).at(y);
}