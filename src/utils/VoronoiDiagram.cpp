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
    for (auto& cell : cells)
        for (auto edgeIt = cell.edges.begin(); edgeIt != cell.edges.end(); edgeIt++) {
            auto nextEdgeIt = edgeIt + 1;
            if (nextEdgeIt == cell.edges.end())
                nextEdgeIt = cell.edges.begin();
            CVector3 startVertex = edgeIt->GetEnd();
            CVector3 endVertex = nextEdgeIt->GetStart();

            if (!areVectorsEqual(startVertex, endVertex)) {
                LOG << "Trying to connect (" << startVertex << ", " << endVertex << ")...\n";
                if (areOnSameSide(startVertex, endVertex)) {
                    LOG << "\tAdd edge (" << startVertex << ", " << endVertex << ")\n";
                    edgeIt = cell.edges.insert(nextEdgeIt, CRay3(startVertex, endVertex));
                }
                else {
                    bool areOnOppositeSides =
                        (startVertex.GetX() == arenaLimits.GetMin().GetX() && endVertex.GetX() == arenaLimits.GetMax().GetX()) ||
                        (startVertex.GetX() == arenaLimits.GetMax().GetX() && endVertex.GetX() == arenaLimits.GetMin().GetX()) ||
                        (startVertex.GetY() == arenaLimits.GetMin().GetY() && endVertex.GetY() == arenaLimits.GetMax().GetY()) ||
                        (startVertex.GetY() == arenaLimits.GetMax().GetY() && endVertex.GetY() == arenaLimits.GetMin().GetY());

                    if (!areOnOppositeSides) {
                        CVector3 corner;
                        corner.SetZ(diagramLiftOnZ);

                        LOG << "Are not on opposite sides!\n";
                        if (startVertex.GetX() == arenaLimits.GetMin().GetX() ||
                            startVertex.GetX() == arenaLimits.GetMax().GetX())
                            corner.SetX(startVertex.GetX());
                        else if (endVertex.GetX() == arenaLimits.GetMin().GetX() ||
                                 endVertex.GetX() == arenaLimits.GetMax().GetX())
                            corner.SetX(endVertex.GetX());
                        else {
                            std::stringstream msg;
                            msg << "None of vertices lay on X boundry! (" << startVertex << ", " << endVertex << ")\n";
                            msg << "Diff X: " << startVertex.GetX() - endVertex.GetX() << "\n";
                            msg << "Diff Y: " << startVertex.GetY() - endVertex.GetY() << "\n";
                            THROW_ARGOSEXCEPTION(msg.str());
                        }

                        if (startVertex.GetY() == arenaLimits.GetMin().GetY() ||
                            startVertex.GetY() == arenaLimits.GetMax().GetY())
                            corner.SetY(startVertex.GetY());
                        else if (endVertex.GetY() == arenaLimits.GetMin().GetY() ||
                                 endVertex.GetY() == arenaLimits.GetMax().GetY())
                            corner.SetY(endVertex.GetY());
                        else {
                            std::stringstream msg;
                            msg << "None of vertices lay on Y boundry! (" << startVertex << ", " << endVertex << ")\n";
                            msg << "Diff X: " << startVertex.GetX() - endVertex.GetX() << "\n";
                            msg << "Diff Y: " << startVertex.GetY() - endVertex.GetY() << "\n";
                            THROW_ARGOSEXCEPTION(msg.str());
                        }

                        LOG << "\tAdd edge (" << startVertex << ", " << corner << ")\n";
                        edgeIt = cell.edges.insert(nextEdgeIt, CRay3(startVertex, corner));

                        LOG << "\tAdd edge (" << corner << ", " << endVertex << ")\n";
                        edgeIt = cell.edges.insert(edgeIt + 1, CRay3(corner, endVertex));
                    }
                    else {
                        CVector3 leftSide(arenaLimits.GetMin().GetX(), 0, diagramLiftOnZ);
                        CVector3 rightSide(arenaLimits.GetMax().GetX(), 0, diagramLiftOnZ);
                        CVector3 bottomSide(0, arenaLimits.GetMin().GetY(), diagramLiftOnZ);
                        CVector3 upSide(0, arenaLimits.GetMax().GetY(), diagramLiftOnZ);

                        CVector3 upperLeftCorner(leftSide.GetX(), upSide.GetY(), diagramLiftOnZ);
                        CVector3 upperRightCorner(rightSide.GetX(), upSide.GetY(), diagramLiftOnZ);
                        CVector3 bottomLeftCorner(leftSide.GetX(), bottomSide.GetY(), diagramLiftOnZ);
                        CVector3 bottomRightCorner(rightSide.GetX(), bottomSide.GetY(), diagramLiftOnZ);

                        CVector3 corners[2];
                        corners[0].SetZ(diagramLiftOnZ);
                        corners[1].SetZ(diagramLiftOnZ);

                        if (areOnSameSide(startVertex, leftSide)) {
                            assert(areOnSameSide(endVertex, rightSide));

                            corners[0].SetX(startVertex.GetX());
                            corners[1].SetX(endVertex.GetX());

                            Real sumOfDistancesFromUpSide =
                                    (startVertex - upperLeftCorner).SquareLength() +
                                    (endVertex - upperRightCorner).SquareLength();
                            Real sumOfDistancesFromBottomSide =
                                    (startVertex - bottomLeftCorner).SquareLength() +
                                    (endVertex - bottomRightCorner).SquareLength();
                            if (sumOfDistancesFromUpSide < sumOfDistancesFromBottomSide) {
                                corners[0].SetY(upSide.GetY());
                                corners[1].SetY(upSide.GetY());
                            }
                            else {
                                corners[0].SetY(bottomSide.GetY());
                                corners[1].SetY(bottomSide.GetY());
                            }
                        }
                        else if (areOnSameSide(startVertex, rightSide)) {
                            assert(areOnSameSide(endVertex, leftSide));

                            corners[0].SetX(startVertex.GetX());
                            corners[1].SetX(endVertex.GetX());

                            Real sumOfDistancesFromUpSide =
                                    (startVertex - upperRightCorner).SquareLength() +
                                    (endVertex - upperLeftCorner).SquareLength();
                            Real sumOfDistancesFromBottomSide =
                                    (startVertex - bottomRightCorner).SquareLength() +
                                    (endVertex - bottomLeftCorner).SquareLength();
                            if (sumOfDistancesFromUpSide < sumOfDistancesFromBottomSide) {
                                corners[0].SetY(upSide.GetY());
                                corners[1].SetY(upSide.GetY());
                            }
                            else {
                                corners[0].SetY(bottomSide.GetY());
                                corners[1].SetY(bottomSide.GetY());
                            }
                        }
                        else if (areOnSameSide(startVertex, bottomSide)) {
                            assert(areOnSameSide(endVertex, upSide));

                            corners[0].SetY(startVertex.GetY());
                            corners[1].SetY(endVertex.GetY());

                            Real sumOfDistancesFromLeftSide =
                                    (startVertex - bottomLeftCorner).SquareLength() +
                                    (endVertex - upperLeftCorner).SquareLength();
                            Real sumOfDistancesFromRightSide =
                                    (startVertex - bottomRightCorner).SquareLength() +
                                    (endVertex - upperRightCorner).SquareLength();
                            if (sumOfDistancesFromLeftSide < sumOfDistancesFromRightSide) {
                                corners[0].SetX(leftSide.GetX());
                                corners[1].SetX(leftSide.GetX());
                            }
                            else {
                                corners[0].SetX(rightSide.GetX());
                                corners[1].SetX(rightSide.GetX());
                            }
                        }
                        else if (areOnSameSide(startVertex, upSide)) {
                            assert(areOnSameSide(endVertex, bottomSide));

                            corners[0].SetY(startVertex.GetY());
                            corners[1].SetY(endVertex.GetY());

                            Real sumOfDistancesFromLeftSide =
                                    (startVertex - upperLeftCorner).SquareLength() +
                                    (endVertex - bottomLeftCorner).SquareLength();
                            Real sumOfDistancesFromRightSide =
                                    (startVertex - upperRightCorner).SquareLength() +
                                    (endVertex - bottomRightCorner).SquareLength();
                            if (sumOfDistancesFromLeftSide < sumOfDistancesFromRightSide) {
                                corners[0].SetX(leftSide.GetX());
                                corners[1].SetX(leftSide.GetX());
                            }
                            else {
                                corners[0].SetX(rightSide.GetX());
                                corners[1].SetX(rightSide.GetX());
                            }
                        }
                        else {
                            THROW_ARGOSEXCEPTION("Start vertex is not on any side!");
                        }

                        LOG << "\tAdd edge (" << startVertex << ", " << corners[0] << ")\n";
                        edgeIt = cell.edges.insert(nextEdgeIt, CRay3(startVertex, corners[0]));

                        LOG << "\tAdd edge (" << corners[0] << ", " << corners[1] << ")\n";
                        edgeIt = cell.edges.insert(edgeIt+1, CRay3(corners[0], corners[1]));

                        LOG << "\tAdd edge (" << corners[1] << ", " << endVertex << ")\n";
                        edgeIt = cell.edges.insert(edgeIt+1, CRay3(corners[1], endVertex));
                    }
                }
            }
        }
}

bool VoronoiDiagram::areOnSameSide(const CVector3 &a, const CVector3 &b) const {
    const double epsilon = 5e-14;
    return fabs(a.GetX() - b.GetX()) < epsilon || fabs(a.GetY() - b.GetY()) < epsilon;
}

bool VoronoiDiagram::areVectorsEqual(const CVector3 &a, const CVector3 &b) const {
    const double epsilon = 5e-14;
    double diffs[3];
    diffs[0] = fabs(a.GetX() - b.GetX());
    diffs[1] = fabs(a.GetY() - b.GetY());
    diffs[2] = fabs(a.GetZ() - b.GetZ());
    return diffs[0] < epsilon && diffs[1] < epsilon && diffs[2] < epsilon;
}

argos::CVector3 VoronoiDiagram::ToVector3(const Vertex& point) const {
    Real x = point.x() / scaleVectorToMilimeters;
    Real y = point.y() / scaleVectorToMilimeters;
    return CVector3(x, y, diagramLiftOnZ);
}

void VoronoiDiagram::updateEdges(const Diagram& diagram) {
    LOG << "Boost voronoi has " << diagram.cells().size() << " cells\n";
    for (auto& cell : diagram.cells()) {
        const voronoi_diagram<Real>::edge_type* edge = cell.incident_edge();
        assert(edge != nullptr);
        assert(edge->is_linear()); // For points all edges should be linear
        cells.emplace_back();
        auto& lastCell = cells.at(cells.size() - 1);
        do {
            if (edge->is_primary()) {
                auto voronoiEdge = ToVoronoiEdge(*edge);
                try {
                    voronoiEdge = getRayBoundedToArena(voronoiEdge);
                    lastCell.edges.push_back(std::move(voronoiEdge));
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

std::vector<argos::CVector3> VoronoiDiagram::getVertices() const {
    std::vector<argos::CVector3> vertices;
    for (const auto& cell : cells)
        for (const auto& edge : cell.edges)
            vertices.emplace_back(edge.GetEnd());
    return vertices;
}

std::vector<CRay3> VoronoiDiagram::getEdges() const {
    std::vector<CRay3> edges;
    for (const auto& cell : cells)
        for (const auto& edge : cell.edges)
            edges.emplace_back(edge);
    return edges;
}
