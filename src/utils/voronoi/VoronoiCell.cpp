#include "VoronoiCell.h"
#include <assert.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace std;
using namespace argos;

VoronoiCell::VoronoiCell(Seed seed, Real diagramLiftOnZ)
    : seed(seed)
    , diagramLiftOnZ(diagramLiftOnZ)
{}

void VoronoiCell::addEdge(CRay3 edge) {
//    CVector3 max = roughCellBoundaries.GetMax();
//    CVector3 min = roughCellBoundaries.GetMin();
//
//    updateMinMax(edge.GetStart(), max, min);
//    updateMinMax(edge.GetEnd(), max, min);
//
//    roughCellBoundaries.Set(min, max);

    edges.push_back(edge);
}

void VoronoiCell::finish() {
    using namespace boost::geometry;

    boostPolygon.clear();

    vector<BoostPoint> points;
    if (edges.size() > 0) {
        auto& vertex = edges.at(0).GetStart();
        points.emplace_back(vertex.GetX(), vertex.GetY());
    }

    for (auto& edge : edges) {
        auto& vertex = edge.GetEnd();
        points.emplace_back(vertex.GetX(), vertex.GetY());
    }
    assign_points(boostPolygon, points);
}

void VoronoiCell::updateMinMax(const CVector3& point, CVector3& max, CVector3& min) const {
    max.SetX( std::max(point.GetX(), max.GetX()) );
    min.SetX( std::min(point.GetX(), min.GetX()) );

    max.SetY( std::max(point.GetY(), max.GetY()) );
    min.SetY( std::min(point.GetY(), min.GetY()) );

    max.SetZ( std::max(point.GetZ(), max.GetZ()) );
    min.SetZ( std::min(point.GetZ(), min.GetZ()) );
}

// TODO: Refactor this!
void VoronoiCell::fillMissingEdges(const CRange<CVector3>& limits) {
    for (auto edgeIt = edges.begin(); edgeIt != edges.end(); edgeIt++) {
        auto nextEdgeIt = edgeIt + 1;
        if (nextEdgeIt == edges.end())
            nextEdgeIt = edges.begin();
        CVector3 startVertex = edgeIt->GetEnd();
        CVector3 endVertex = nextEdgeIt->GetStart();

        if (!areVectorsEqual(startVertex, endVertex)) {
            if (areOnSameSide(startVertex, endVertex)) {
                edgeIt = edges.insert(nextEdgeIt, CRay3(startVertex, endVertex));
            }
            else {
                if (!areOnOppositeSides(startVertex, endVertex, limits)) {
                    CVector3 corner;
                    corner.SetX(getHorizontalLimit(startVertex, endVertex, limits));
                    corner.SetY(getVerticalLimit(startVertex, endVertex, limits));
                    corner.SetZ(diagramLiftOnZ);

                    edgeIt = edges.insert(nextEdgeIt, CRay3(startVertex, corner));
                    edgeIt = edges.insert(edgeIt + 1, CRay3(corner, endVertex));


                    finish();
                    // Assertion
                    if (!isInside(seed.position)) {
                        edgeIt = edges.erase(edgeIt-1, edgeIt+1);

                        std::array<CVector3,3> corners;
                        corner.SetX(-corner.GetX());
                        corners[0] = corner;
                        corner.SetY(-corner.GetY());
                        corners[1] = corner;
                        corner.SetX(-corner.GetX());
                        corners[2] = corner;

                        if (!areOnSameSide(startVertex, corners[0])) {
                            corners[2] = corners[0];
                            corners[0] = corner;
                        }

                        edgeIt = edges.insert(edgeIt, CRay3(startVertex, corners[0]));
                        edgeIt = edges.insert(edgeIt+1, CRay3(corners[0], corners[1]));
                        edgeIt = edges.insert(edgeIt+1, CRay3(corners[1], corners[2]));
                        edgeIt = edges.insert(edgeIt+1, CRay3(corners[2], endVertex));
                    }
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

                    array<CVector3,2> corners;
                    corners[0].SetZ(diagramLiftOnZ);
                    corners[1].SetZ(diagramLiftOnZ);

                    if (areOnSameSide(startVertex, leftSide) && areOnSameSide(endVertex, rightSide)) {
                        array<CVector3,2> sideCorners = {upperLeftCorner, bottomLeftCorner};
                        Real y = getClosestHorizontal(startVertex, endVertex, sideCorners);
                        corners[0].SetX(startVertex.GetX());
                        corners[1].SetX(endVertex.GetX());
                        corners[0].SetY(y);
                        corners[1].SetY(y);
                    }
                    else if (areOnSameSide(startVertex, rightSide) && areOnSameSide(endVertex, leftSide)) {
                        array<CVector3,2> sideCorners = {upperRightCorner, bottomRightCorner};
                        Real y = getClosestHorizontal(startVertex, endVertex, sideCorners);
                        corners[0].SetX(startVertex.GetX());
                        corners[1].SetX(endVertex.GetX());
                        corners[0].SetY(y);
                        corners[1].SetY(y);
                    }
                    else if (areOnSameSide(startVertex, bottomSide) && areOnSameSide(endVertex, upSide)) {
                        array<CVector3,2> sideCorners = {bottomLeftCorner, bottomRightCorner};
                        Real x = getClosestVertical(startVertex, endVertex, sideCorners);
                        corners[0].SetX(x);
                        corners[1].SetX(x);
                        corners[0].SetY(startVertex.GetY());
                        corners[1].SetY(endVertex.GetY());
                    }
                    else if (areOnSameSide(startVertex, upSide) && areOnSameSide(endVertex, bottomSide)) {
                        array<CVector3,2> sideCorners = {upperLeftCorner, upperRightCorner};
                        Real x = getClosestVertical(startVertex, endVertex, sideCorners);
                        corners[0].SetX(x);
                        corners[1].SetX(x);
                        corners[0].SetY(startVertex.GetY());
                        corners[1].SetY(endVertex.GetY());
                    }
                    else {
                        THROW_ARGOSEXCEPTION("Start vertex is not on any side!");
                    }

                    edgeIt = edges.insert(nextEdgeIt, CRay3(startVertex, corners[0]));
                    edgeIt = edges.insert(edgeIt+1, CRay3(corners[0], corners[1]));
                    edgeIt = edges.insert(edgeIt+1, CRay3(corners[1], endVertex));
                    finish();

                    // Assertion
                    if (!isInside(seed.position)) {
                        edgeIt = edges.erase(edgeIt-2, edgeIt+1);
                        if (corners[0].GetX() == corners[1].GetX()) {
                            corners[0].SetX(-corners[0].GetX());
                            corners[1].SetX(-corners[1].GetX());
                        }
                        else if (corners[0].GetY() == corners[1].GetY()) {
                            corners[0].SetY(-corners[0].GetY());
                            corners[1].SetY(-corners[1].GetY());
                        }
                        edgeIt = edges.insert(edgeIt, CRay3(startVertex, corners[0]));
                        edgeIt = edges.insert(edgeIt+1, CRay3(corners[0], corners[1]));
                        edgeIt = edges.insert(edgeIt+1, CRay3(corners[1], endVertex));
                    }
                }
            }
        }
    }
    finish();
}

Real VoronoiCell::getClosestVertical(const CVector3& startVertex, const CVector3& endVertex,
                                     const array<argos::CVector3,2> corners) const {
    array<CVector3,2> complementaryCorners = {corners[0], corners[1]};
    complementaryCorners[0].SetY(-corners[0].GetY());
    complementaryCorners[1].SetY(-corners[1].GetY());
    return getClosestCorner(startVertex, endVertex, corners, complementaryCorners).GetX();
}

Real VoronoiCell::getClosestHorizontal(const CVector3& startVertex, const CVector3& endVertex,
                                       const array<CVector3,2> corners) const {
    array<CVector3,2> complementaryCorners = {corners[0], corners[1]};
    complementaryCorners[0].SetX(-corners[0].GetX());
    complementaryCorners[1].SetX(-corners[1].GetX());
    return getClosestCorner(startVertex, endVertex, corners, complementaryCorners).GetY();
}

CVector3 VoronoiCell::getClosestCorner(const CVector3& startVertex, const CVector3& endVertex,
                                       const array<CVector3, 2> corners, const array<CVector3, 2>
                                       complementaryCorners) const {
    Real distanceFromFirstSide= (startVertex - corners[0]).SquareLength();
    distanceFromFirstSide += (endVertex - complementaryCorners[0]).SquareLength();

    Real distanceFromSecondSide= (startVertex - corners[1]).SquareLength();
    distanceFromSecondSide += (endVertex - complementaryCorners[1]).SquareLength();

    return (distanceFromFirstSide < distanceFromSecondSide) ? corners[0] : corners[1];
}

Real VoronoiCell::getVerticalLimit(const CVector3& startVertex, const CVector3& endVertex,
                                          const CRange<CVector3>& limits) const {
    if (isVerticalLimit(startVertex, limits))
        return startVertex.GetY();
    else if (isVerticalLimit(endVertex, limits))
        return endVertex.GetY();
    THROW_ARGOSEXCEPTION(
        "None of vertices lay on Y boundry! " + getVerticesLog(startVertex, endVertex, limits));
}

Real VoronoiCell::getHorizontalLimit(const CVector3& startVertex, const CVector3& endVertex,
                                     const CRange<CVector3>& limits) const {
    if (isHorizontalLimit(startVertex, limits))
        return startVertex.GetX();
    else if (isHorizontalLimit(endVertex, limits))
        return endVertex.GetX();
    THROW_ARGOSEXCEPTION(
        "None of vertices lay on X boundry! " + getVerticesLog(startVertex, endVertex, limits));
}

bool VoronoiCell::isVerticalLimit(const CVector3& vertex, const CRange<CVector3>& limits) const {
    return vertex.GetY() == limits.GetMin().GetY() ||
                        vertex.GetY() == limits.GetMax().GetY();
}

string VoronoiCell::getVerticesLog(const CVector3& a, const CVector3& b, const CRange<CVector3>& limits) const {
    stringstream s;
    s << "(" << a << ", " << b << ")\n"
        << "start - min: " << a.GetX() - limits.GetMin().GetX() << "\n"
        << "start - max: " << a.GetX() - limits.GetMax().GetX() << "\n"
        << "end - max: " << b.GetX() - limits.GetMin().GetX() << "\n"
        << "end - max: " << b.GetX() - limits.GetMax().GetX() << "\n";
    return s.str();
}

bool VoronoiCell::isHorizontalLimit(const CVector3& vertex, const CRange<CVector3>& limits) const {
    return vertex.GetX() == limits.GetMin().GetX() ||
                        vertex.GetX() == limits.GetMax().GetX();
}

bool VoronoiCell::areOnOppositeSides(const CVector3& a, const CVector3& b, const CRange<CVector3>& limits) const {
    return (a.GetX() == limits.GetMin().GetX() && b.GetX() == limits.GetMax().GetX()) ||
           (a.GetX() == limits.GetMax().GetX() && b.GetX() == limits.GetMin().GetX()) ||
           (a.GetY() == limits.GetMin().GetY() && b.GetY() == limits.GetMax().GetY()) ||
           (a.GetY() == limits.GetMax().GetY() && b.GetY() == limits.GetMin().GetY());
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
//    if (!roughCellBoundaries.WithinMinBoundIncludedMaxBoundIncluded(point))
//        return false;
    BoostPoint p(point.GetX(), point.GetY());
    return boost::geometry::within(p, boostPolygon);
}

const vector<CRay3>& VoronoiCell::getEdges() const {
    return edges;
}
