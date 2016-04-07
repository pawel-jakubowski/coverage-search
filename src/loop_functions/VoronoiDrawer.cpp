#include <argos3/core/simulator/simulator.h>
#include "VoronoiDrawer.h"

using namespace argos;

VoronoiDrawer::VoronoiDrawer()
    : voronoi(dynamic_cast<CalculateVoronoi&>(CSimulator::GetInstance().GetLoopFunctions())) {}

void VoronoiDrawer::DrawInWorld() {
    drawVertices();

    auto voronoiEdges = voronoi.getEdges();
    LOG << "Draw " << voronoiEdges.size() << " edges" "\n";
    for (auto& edge : voronoiEdges)
        DrawRay(CRay3(edge.begin, edge.end));
}

void VoronoiDrawer::drawVertices() {
    auto voronoiVertices = voronoi.getVertices();
    LOG << "Draw " << voronoiVertices.size() << " vertices" "\n";
    for (auto& vertex : voronoiVertices)
        DrawPoint(vertex, vertexColor, vertexSize);
}

REGISTER_QTOPENGL_USER_FUNCTIONS(VoronoiDrawer, "draw_voronoi")
