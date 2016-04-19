#include "MbfoDrawer.h"

using namespace argos;

MbfoDrawer::MbfoDrawer()
        : mbfo(dynamic_cast<MbfoLoopFunction&>(CSimulator::GetInstance().GetLoopFunctions())) {}

void MbfoDrawer::DrawInWorld() {
    mbfo.update();
    drawVoronoi();
    drawGrid();
}

void MbfoDrawer::drawVoronoi() {
    drawVertices();
    drawEdges();
}

void MbfoDrawer::drawGrid() {
    auto grid = mbfo.getCoverageGrid();
    for (auto& row : grid)
        for (auto& cell : row)
            drawCell(cell);
}

void MbfoDrawer::drawCell(const CoverageGrid::Cell &cell) {
    std::vector<CVector2> points;
    const auto& start = cell.edges.front().GetStart();
    points.emplace_back(start.GetX(), start.GetY());
    for (auto& edge : cell.edges)
        points.emplace_back(edge.GetEnd().GetX(), edge.GetEnd().GetY());
    points.pop_back();
    DrawPolygon(
            CVector3(0,0,start.GetZ()),
            CQuaternion(CRadians::ZERO, CVector3(1,0,0)),
            points,
            gridColor,
            cell.isCovered);
}

void MbfoDrawer::drawEdges() {
    auto voronoiEdges = mbfo.getVoronoiEdges();
    LOG << "Draw " << voronoiEdges.size() << " edges" "\n";
    for (auto& edge : voronoiEdges)
        DrawRay(edge);
}

void MbfoDrawer::drawVertices() {
    auto voronoiVertices = mbfo.getVoronoiVertices();
    LOG << "Draw " << voronoiVertices.size() << " vertices" "\n";
    for (auto& vertex : voronoiVertices)
        DrawPoint(vertex, voronoiVertexColor, vertexSize);
}

REGISTER_QTOPENGL_USER_FUNCTIONS(MbfoDrawer, "draw_mbfo")
