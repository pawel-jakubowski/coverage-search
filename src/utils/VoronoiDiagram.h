#pragma once

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/ray3.h>
#include <boost/polygon/point_data.hpp>
#include <boost/polygon/voronoi.hpp>

class VoronoiDiagram {
public:
    void calculate(std::vector<argos::CVector3> points);
    void setArenaLimits(argos::CRange<argos::CVector3> limits);
    std::vector<argos::CVector3> getVertices() const;
    std::vector<argos::CRay3> getEdges() const;

private:
    using CoordinateType = argos::Real;
    using Point = boost::polygon::point_data<CoordinateType>;
    using Diagram = boost::polygon::voronoi_diagram<CoordinateType>;
    using Vertex = boost::polygon::voronoi_vertex<CoordinateType>;
    using Edge = boost::polygon::voronoi_edge<CoordinateType>;

    const int scaleVectorToMilimeters = 100000;
    const argos::Real diagramLiftOnZ = 0.1f;
    argos::CRange<argos::CVector3> arenaLimits;
    std::vector<Point> boostPoints;
    std::vector<argos::CVector3> vertices;
    std::vector<argos::CRay3> edges;

    void reset();
    void updateVoronoiDiagram();
    void updateVertices(const Diagram& diagram);
    void updateEdges(const Diagram& diagram);

    Point ToPoint(const argos::CVector3& vec) const;
    argos::CVector3 ToVector3(const Vertex& vertex) const;
    argos::CRay3 ToVoronoiEdge(const Edge& edge) const;
    argos::CRay3 getRayBoundedToArena(const argos::CRay3 &ray) const;
};


