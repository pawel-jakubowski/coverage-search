#pragma once

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/ray3.h>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>


class VoronoiCell {
public:
    struct CoverageCell {
        int x;
        int y;
        CoverageCell(int x, int y) : x(x), y(y) {}
    };

    struct Seed {
        std::string id;
        argos::CVector3 position;
    };

    Seed seed;
    std::vector<CoverageCell> coverageCells;

    VoronoiCell(Seed seed, argos::Real diagramLiftOnZ = 0.02f);
    void addEdge(argos::CRay3 edge);
    void finish();
    void fillMissingEdges(const argos::CRange<argos::CVector3>& limits);
    bool isInside(argos::CVector3 point) const;
    const std::vector<argos::CRay3>& getEdges() const;

private:
    using BoostPoint = boost::geometry::model::d2::point_xy<double>;
    using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

    std::vector<argos::CRay3> edges;
    const argos::Real diagramLiftOnZ;
    argos::CRange<argos::CVector3> roughCellBoundaries;
    BoostPolygon boostPolygon;

    bool areVectorsEqual(const argos::CVector3 &a, const argos::CVector3 &b) const;
    bool areOnSameSide(const argos::CVector3 &a, const argos::CVector3 &b) const;

    void updateMinMax(const argos::CVector3& point, argos::CVector3& max, argos::CVector3& min) const;
};


