#include "VoronoiCell.h"
#include <assert.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace argos;

VoronoiCell::VoronoiCell(Seed seed, Real diagramLiftOnZ)
    : seed(seed)
    , diagramLiftOnZ(diagramLiftOnZ)
{}


// TODO: Refactor this!
void VoronoiCell::fillMissingEdges(const CRange<CVector3>& limits) {
    for (auto edgeIt = edges.begin(); edgeIt != edges.end(); edgeIt++) {
        auto nextEdgeIt = edgeIt + 1;
        if (nextEdgeIt == edges.end())
            nextEdgeIt = edges.begin();
        CVector3 startVertex = edgeIt->GetEnd();
        CVector3 endVertex = nextEdgeIt->GetStart();

        if (!areVectorsEqual(startVertex, endVertex)) {
            //LOG << "Trying to connect (" << startVertex << ", " << endVertex << ")...\n";
            if (areOnSameSide(startVertex, endVertex)) {
                //LOG << "\tAdd edge (" << startVertex << ", " << endVertex << ")\n";
                edgeIt = edges.insert(nextEdgeIt, CRay3(startVertex, endVertex));
            }
            else {
                bool areOnOppositeSides =
                        (startVertex.GetX() == limits.GetMin().GetX() && endVertex.GetX() == limits.GetMax().GetX()) ||
                        (startVertex.GetX() == limits.GetMax().GetX() && endVertex.GetX() == limits.GetMin().GetX()) ||
                        (startVertex.GetY() == limits.GetMin().GetY() && endVertex.GetY() == limits.GetMax().GetY()) ||
                        (startVertex.GetY() == limits.GetMax().GetY() && endVertex.GetY() == limits.GetMin().GetY());

                if (!areOnOppositeSides) {
                    CVector3 corner;
                    corner.SetZ(diagramLiftOnZ);

                    //LOG << "Are not on opposite sides!\n";
                    if (startVertex.GetX() == limits.GetMin().GetX() ||
                        startVertex.GetX() == limits.GetMax().GetX())
                        corner.SetX(startVertex.GetX());
                    else if (endVertex.GetX() == limits.GetMin().GetX() ||
                             endVertex.GetX() == limits.GetMax().GetX())
                        corner.SetX(endVertex.GetX());
                    else {
                        std::stringstream msg;
                        msg << "None of vertices lay on X boundry! (" << startVertex << ", " << endVertex << ")\n";
                        msg << "start - min: " << startVertex.GetX() - limits.GetMin().GetX() << "\n";
                        msg << "start - max: " << startVertex.GetX() - limits.GetMax().GetX() << "\n";
                        msg << "end - max: " << endVertex.GetX() - limits.GetMin().GetX() << "\n";
                        msg << "end - max: " << endVertex.GetX() - limits.GetMax().GetX() << "\n";
                        THROW_ARGOSEXCEPTION(msg.str());
                    }

                    if (startVertex.GetY() == limits.GetMin().GetY() ||
                        startVertex.GetY() == limits.GetMax().GetY())
                        corner.SetY(startVertex.GetY());
                    else if (endVertex.GetY() == limits.GetMin().GetY() ||
                             endVertex.GetY() == limits.GetMax().GetY())
                        corner.SetY(endVertex.GetY());
                    else {
                        std::stringstream msg;
                        msg << "None of vertices lay on Y boundry! (" << startVertex << ", " << endVertex << ")\n";
                        msg << "start - min: " << startVertex.GetY() - limits.GetMin().GetY() << "\n";
                        msg << "start - max: " << startVertex.GetY() - limits.GetMax().GetY() << "\n";
                        msg << "end - max: " << endVertex.GetY() - limits.GetMin().GetY() << "\n";
                        msg << "end - max: " << endVertex.GetY() - limits.GetMax().GetY() << "\n";
                        THROW_ARGOSEXCEPTION(msg.str());
                    }

                    //LOG << "\tAdd edge (" << startVertex << ", " << corner << ")\n";
                    edgeIt = edges.insert(nextEdgeIt, CRay3(startVertex, corner));

                    //LOG << "\tAdd edge (" << corner << ", " << endVertex << ")\n";
                    edgeIt = edges.insert(edgeIt + 1, CRay3(corner, endVertex));
                }
                else {
                    CVector3 leftSide(limits.GetMin().GetX(), 0, diagramLiftOnZ);
                    CVector3 rightSide(limits.GetMax().GetX(), 0, diagramLiftOnZ);
                    CVector3 bottomSide(0, limits.GetMin().GetY(), diagramLiftOnZ);
                    CVector3 upSide(0, limits.GetMax().GetY(), diagramLiftOnZ);

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

                    //LOG << "\tAdd edge (" << startVertex << ", " << corners[0] << ")\n";
                    edgeIt = edges.insert(nextEdgeIt, CRay3(startVertex, corners[0]));

                    //LOG << "\tAdd edge (" << corners[0] << ", " << corners[1] << ")\n";
                    edgeIt = edges.insert(edgeIt+1, CRay3(corners[0], corners[1]));

                    //LOG << "\tAdd edge (" << corners[1] << ", " << endVertex << ")\n";
                    edgeIt = edges.insert(edgeIt+1, CRay3(corners[1], endVertex));
                }
            }
        }
    }
}

bool VoronoiCell::areOnSameSide(const CVector3 &a, const CVector3 &b) const {
    const double epsilon = 1e-10;
    return fabs(a.GetX() - b.GetX()) < epsilon || fabs(a.GetY() - b.GetY()) < epsilon;
}

bool VoronoiCell::areVectorsEqual(const CVector3 &a, const CVector3 &b) const {
    const double epsilon = 1e-10;
    double diffs[3];
    diffs[0] = fabs(a.GetX() - b.GetX());
    diffs[1] = fabs(a.GetY() - b.GetY());
    diffs[2] = fabs(a.GetZ() - b.GetZ());
    return diffs[0] < epsilon && diffs[1] < epsilon && diffs[2] < epsilon;
}

bool VoronoiCell::isInside(CVector3 point) const {
    using namespace boost::geometry;
    using Point = model::d2::point_xy<double>;
    using Polygon = model::polygon<Point>;

    Polygon polygon;
    std::vector<Point> points;
    if (edges.size() > 0) {
        auto& vertex = edges.at(0).GetStart();
        points.emplace_back(vertex.GetX(), vertex.GetY());
    }

    for (auto& edge : edges) {
        auto& vertex = edge.GetEnd();
        points.emplace_back(vertex.GetX(), vertex.GetY());
    }
    assign_points(polygon, points);

    Point p(point.GetX(), point.GetY());
    return within(p, polygon);
}
