#include "CellularDrawer.h"

using namespace std;
using namespace argos;

CellularDrawer::CellularDrawer()
    : loopFnc(dynamic_cast<CellularDecomposition&>(CSimulator::GetInstance().GetLoopFunctions()))
{}

void CellularDrawer::DrawInWorld() {
    drawCoverageGrid();
    drawTaskCells();
}

void CellularDrawer::drawTaskCells() {
    auto cells = loopFnc.getTaskCells();
    Real liftOnZ = 0.011f;
    for (auto& cell : cells)
        drawCell(cell.getLimits(), liftOnZ);
}

void CellularDrawer::drawCell(CRange<CVector2> limits, Real liftOnZ) {
    vector<CVector2> points = {
            {limits.GetMin().GetX(), limits.GetMin().GetY()},
            {limits.GetMin().GetX(), limits.GetMax().GetY()},
            {limits.GetMax().GetX(), limits.GetMax().GetY()},
            {limits.GetMax().GetX(), limits.GetMin().GetY()},
        };
    DrawPolygon(
            CVector3(0, 0, liftOnZ),
            CQuaternion(CRadians::ZERO, CVector3(1, 0, 0)),
            points,
            CColor::RED,
            false,
            4);
}

void CellularDrawer::drawCoverageGrid() {
    auto grid = loopFnc.getCoverageGrid();
    for (int i = 0; i < grid.getGrid().size(); i++)
        for (int j = 0; j < grid.getGrid().size(); j++)
            drawCoverageCell(grid.getGrid().at(i).at(j));
}

void CellularDrawer::drawCoverageCell(const CoverageGrid::Cell& cell) {
    std::vector<CVector2> points;
    const auto& start = cell.edges.front().GetStart();
    points.emplace_back(start.GetX(), start.GetY());
    for (auto& edge : cell.edges)
        points.emplace_back(edge.GetEnd().GetX(), edge.GetEnd().GetY());
    points.pop_back();
    auto maxConcentration = loopFnc.maxCellConcentration;
    CColor color;
    auto colorValue
        = gridFloorDiff * (static_cast<double>(cell.concentration) / maxConcentration);
    colorValue += gridColor;
    color = CColor(colorValue, colorValue, colorValue);
    DrawPolygon(
            CVector3(0,0,start.GetZ()),
            CQuaternion(CRadians::ZERO, CVector3(1,0,0)),
            points,
            color,
            true);
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CellularDrawer, "cellular_loop_function_qt")
