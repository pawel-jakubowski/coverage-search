#include "CellularDrawer.h"

using namespace std;
using namespace argos;

CellularDrawer::CellularDrawer()
    : loopFnc(dynamic_cast<CellularDecomposition&>(CSimulator::GetInstance().GetLoopFunctions()))
{}

void CellularDrawer::DrawInWorld() {
    drawGrid();
}

void CellularDrawer::drawGrid() {
    auto grid = loopFnc.getCoverageGrid();
    for (int i = 0; i < grid.getGrid().size(); i++)
        for (int j = 0; j < grid.getGrid().size(); j++)
            drawCell(grid.getGrid().at(i).at(j));
}

void CellularDrawer::drawCell(const CoverageGrid::Cell &cell) {
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
