#include "VoronoiDiagram.h"
#include <utils/math/liang-barsky.h>
#include <argos3/core/utility/logging/argos_log.h>

#include "assert.h"

using namespace std;
using namespace argos;
using namespace boost::polygon;

void VoronoiDiagram::calculate(map<string, CVector3> points) {
    reset();
    for (const auto& idPointPair : points) {
        seeds.push_back(Cell::Seed{idPointPair.first, idPointPair.second});
        boostPoints.push_back(ToPoint(idPointPair.second));
    }
    updateVoronoiDiagram();
}

void VoronoiDiagram::calculate(map<string, CVector3> points, const vector<vector<CoverageCell>>& grid) {
    calculate(move(points));
    for (auto& cell : cells)
        for (int i = 0; i < grid.size(); i++)
            for (int j = 0; j < grid.at(i).size(); j++)
                if (cell.isInside(grid.at(i).at(j).center))
                    cell.coverageCells.emplace_back(i, j);
}

void VoronoiDiagram::reset() {
    seeds.clear();
    boostPoints.clear();
    cells.clear();
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
    updateEdges(voronoiDiagram);
    fillMissingEdges();
}

void VoronoiDiagram::fillMissingEdges() {
    for (auto& cell : cells) {
        cell.fillMissingEdges(arenaLimits);
    }
}

argos::CVector3 VoronoiDiagram::ToVector3(const Vertex& point) const {
    Real x = point.x() / scaleVectorToMilimeters;
    Real y = point.y() / scaleVectorToMilimeters;
    return CVector3(x, y, diagramLiftOnZ);
}

void VoronoiDiagram::updateEdges(const Diagram& diagram) {
    //LOG << "Boost voronoi has " << diagram.cells().size() << " cells\n";
    for (auto& cell : diagram.cells()) {
        assert(cell.contains_point()); // Cell should be created by point seed
        const voronoi_diagram<Real>::edge_type* edge = cell.incident_edge();
        assert(edge->is_linear()); // For points all edges should be linear
        assert(edge != nullptr);
        cells.emplace_back(seeds.at(cell.source_index()), diagramLiftOnZ);
        auto& lastCell = cells.at(cells.size() - 1);
        do {
            if (edge->is_primary()) {
                auto voronoiEdge = ToVoronoiEdge(*edge);
                try {
                    voronoiEdge = getRayBoundedToArena(voronoiEdge);
                    lastCell.addEdge(move(voronoiEdge));
                }
                catch(EdgeNotInArea& e) {
                    // Do not add this edge
                }
            }
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
        Real koef = side / max(fabs(direction.x()), fabs(direction.y()));
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
            end.SetZ(diagramLiftOnZ);
            voronoiEdge.SetEnd(end);
        } else {
            voronoiEdge.SetEnd(ToVector3(*edge.vertex1()));
        }
    }
    return voronoiEdge;
}

CRay3 VoronoiDiagram::getRayBoundedToArena(const CRay3 &ray) const {
    Real startX, startY, endX, endY;
    bool edgeWasClipped = LiangBarsky(
            arenaLimits.GetMin().GetX(),
            arenaLimits.GetMax().GetX(),
            arenaLimits.GetMin().GetY(),
            arenaLimits.GetMax().GetY(),
            ray.GetStart().GetX(), ray.GetStart().GetY(),
            ray.GetEnd().GetX(), ray.GetEnd().GetY(),
            startX, startY, endX, endY);

    if (!edgeWasClipped)
        throw EdgeNotInArea();
    CRay3 boundedRay(CVector3(startX, startY, diagramLiftOnZ),
                     CVector3(endX, endY, diagramLiftOnZ));\
    return boundedRay;
}

void VoronoiDiagram::setArenaLimits(CRange<CVector3> limits) {
    arenaLimits = limits;
}

vector<CVector3> VoronoiDiagram::getVertices() const {
    vector<argos::CVector3> vertices;
    for (const auto& cell : cells)
        for (const auto& edge : cell.getEdges())
            vertices.emplace_back(edge.GetEnd());
    return vertices;
}

vector<CRay3> VoronoiDiagram::getEdges() const {
    vector<CRay3> edges;
    for (const auto& cell : cells)
        for (const auto& edge : cell.getEdges())
            edges.emplace_back(edge);
    return edges;
}

const vector<VoronoiDiagram::Cell>& VoronoiDiagram::getCells() const {
    return cells;
}
