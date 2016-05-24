#include "MbfoDrawer.h"

using namespace argos;

MbfoDrawer::MbfoDrawer()
    : mbfo(dynamic_cast<MbfoLoopFunction&>(CSimulator::GetInstance().GetLoopFunctions()))
{}

void MbfoDrawer::DrawInWorld() {
    drawVoronoi();
    drawGrid();
}

void MbfoDrawer::drawVoronoi() {
    auto voronoiCells = mbfo.getVoronoiCells();
    for (auto& cell : voronoiCells) {
//        drawCellId(cell);
        for (auto& edge : cell.getEdges()) {
            drawEdge(edge);
            drawVertex(edge);
        }
    }
}

void MbfoDrawer::drawCellId(const VoronoiDiagram::Cell& cell) {
    auto textPosition = cell.seed.position;
    Real zAxisLift = 0.05f;
    textPosition.SetZ(textPosition.GetZ() + zAxisLift);
    DrawText(textPosition, cell.seed.id, CColor::WHITE);
}

void MbfoDrawer::drawEdge(const CRay3& edge) { DrawRay(edge, CColor::RED, 3.0f); }

void MbfoDrawer::drawVertex(const CRay3& edge) {
    auto &vertex = edge.GetStart();
    DrawPoint(vertex, voronoiVertexColor, vertexSize);
}

void MbfoDrawer::drawGrid() {
    auto grid = mbfo.getCoverageGrid();
    for (auto& row : grid.getGrid())
        for (auto &cell : row)
            drawCell(cell);
}

void MbfoDrawer::drawCell(const CoverageGrid::Cell &cell) {
    std::vector<CVector2> points;
    const auto& start = cell.edges.front().GetStart();
    points.emplace_back(start.GetX(), start.GetY());
    for (auto& edge : cell.edges)
        points.emplace_back(edge.GetEnd().GetX(), edge.GetEnd().GetY());
    points.pop_back();
    auto maxConcentration = mbfo.maxCellConcentration;
    auto color = gridFloorDiff * (static_cast<double>(cell.concentration) / maxConcentration);
    color += gridColor;
    DrawPolygon(
            CVector3(0,0,start.GetZ()),
            CQuaternion(CRadians::ZERO, CVector3(1,0,0)),
            points,
            CColor(color, color, color),
            true);
}

REGISTER_QTOPENGL_USER_FUNCTIONS(MbfoDrawer, "draw_mbfo")
