#include "VoronoiDiagram.h"
#include "liang-barsky.h"
#include <argos3/core/utility/logging/argos_log.h>

#include "assert.h"

using namespace argos;
using namespace boost::polygon;

void VoronoiDiagram::calculate(std::vector<CVector3> points) {
    reset();
    for (const auto& point : points)
        boostPoints.push_back(ToPoint(point));
    updateVoronoiDiagram();
}

void VoronoiDiagram::reset() {
    boostPoints.clear();
    vertices.clear();
    edges.clear();
}

VoronoiDiagram::Point VoronoiDiagram::ToPoint(const CVector3& vec) const {
    VoronoiDiagram::Point point;
    /* Assuming that Vector have meter values */
    point.set(HORIZONTAL, vec.GetX()*scaleVectorToMilimeters);
    point.set(VERTICAL, vec.GetY()*scaleVectorToMilimeters);
    return point;
}

void VoronoiDiagram::updateVoronoiDiagram() {
    Diagram voronoiDiagram;
    construct_voronoi(boostPoints.begin(), boostPoints.end(), &voronoiDiagram);
    updateVertices(voronoiDiagram);
    updateEdges(voronoiDiagram);
}

void VoronoiDiagram::updateVertices(const Diagram& diagram) {
    CVector3 vectorVertex;
    for (auto& vertex : diagram.vertices()) {
        vectorVertex = ToVector3(vertex);
        if (arenaLimits.WithinMinBoundIncludedMaxBoundIncluded(vectorVertex))
            vertices.emplace_back(vectorVertex);
    }
}

argos::CVector3 VoronoiDiagram::ToVector3(const Vertex& point) const {
    Real x = point.x() / scaleVectorToMilimeters;
    Real y = point.y() / scaleVectorToMilimeters;
    return CVector3(x, y, diagramLiftOnZ);
}

void VoronoiDiagram::updateEdges(const Diagram& diagram) {
    for (auto& cell : diagram.cells()) {
        const voronoi_diagram<Real>::edge_type* edge = cell.incident_edge();
        assert(edge != nullptr);
        assert(edge->is_linear()); // For points all edges should be linear
        do {
            if (edge->is_primary())
                edges.push_back(ToVoronoiEdge(*edge));
            edge = edge->next();
        } while (edge != cell.incident_edge());
    }
}

CRay3 VoronoiDiagram::ToVoronoiEdge(const Edge& edge) const {
    CRay3 voronoiEdge;
    if (edge.is_finite()) {
        voronoiEdge.SetStart(ToVector3(*edge.vertex0()));
        voronoiEdge.SetEnd(ToVector3(*edge.vertex1()));
    }
    else {
        const auto& cell1 = *edge.cell();
        const auto& cell2 = *edge.twin()->cell();
        VoronoiDiagram::Point origin, direction;

        VoronoiDiagram::Point p1 = boostPoints.at(cell1.source_index());
        VoronoiDiagram::Point p2 = boostPoints.at(cell2.source_index());
        p1.set(HORIZONTAL, p1.x()/scaleVectorToMilimeters);
        p1.set(VERTICAL, p1.y()/scaleVectorToMilimeters);
        p2.set(HORIZONTAL, p2.x()/scaleVectorToMilimeters);
        p2.set(VERTICAL, p2.y()/scaleVectorToMilimeters);
        origin.x((p1.x() + p2.x()) * 0.5);
        origin.y((p1.y() + p2.y()) * 0.5);
        direction.x(p1.y() - p2.y());
        direction.y(p2.x() - p1.x());

        Real side = arenaLimits.GetMax().GetX()*2;
        Real koef = side / std::max(fabs(direction.x()), fabs(direction.y()));
        if (edge.vertex0() == NULL) {
            CVector3 start;
            start.SetX(origin.x() - (direction.x() * koef));
            start.SetY(origin.y() - (direction.y() * koef));
            start.SetZ(diagramLiftOnZ);
            voronoiEdge.SetStart(start);
        } else {
            voronoiEdge.SetStart(ToVector3(*edge.vertex0()));
        }
        if (edge.vertex1() == NULL) {
            CVector3 end;
            end.SetX(origin.x() + direction.x() * koef);
            end.SetY(origin.y() + direction.y() * koef);
            voronoiEdge.SetEnd(end);
        } else {
            voronoiEdge.SetEnd(ToVector3(*edge.vertex1()));
        }
    }
    return getRayBoundedToArena(voronoiEdge);
}

CRay3 VoronoiDiagram::getRayBoundedToArena(const CRay3 &ray) const {
    Real startX, startY, endX, endY;
    LiangBarsky(
            arenaLimits.GetMin().GetX(),
            arenaLimits.GetMax().GetX(),
            arenaLimits.GetMin().GetY(),
            arenaLimits.GetMax().GetY(),
            ray.GetStart().GetX(), ray.GetStart().GetY(),
            ray.GetEnd().GetX(), ray.GetEnd().GetY(),
            startX, startY, endX, endY);
    CRay3 boundedRay(CVector3(startX, startY, diagramLiftOnZ),
                     CVector3(endX, endY, diagramLiftOnZ));\
    return boundedRay;
}

void VoronoiDiagram::setArenaLimits(CRange<CVector3> limits) {
    arenaLimits = limits;
}

std::vector<argos::CVector3> VoronoiDiagram::getVertices() const {
    return vertices;
}

std::vector<CRay3> VoronoiDiagram::getEdges() const {
    return edges;
}
