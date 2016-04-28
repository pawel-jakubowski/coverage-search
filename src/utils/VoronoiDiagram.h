#pragma once

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/ray3.h>
#include <boost/polygon/point_data.hpp>
#include <boost/polygon/voronoi.hpp>

class VoronoiDiagram {
public:
    struct Cell {
        std::vector<argos::CVector3> vertices;
        std::vector<argos::CRay3> edges;
    };

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

    class EdgeNotInArea : public std::logic_error {
    public:
        EdgeNotInArea() : std::logic_error("Edge not in area") {}
    };

    const int scaleVectorToMilimeters = 100000;
    const argos::Real diagramLiftOnZ = 0.1f;
    argos::CRange<argos::CVector3> arenaLimits;
    std::vector<Point> boostPoints;
    std::vector<Cell> cells;

    void reset();
    void updateVoronoiDiagram();
    void fillMissingEdges();
    void updateEdges(const Diagram& diagram);

    Point ToPoint(const argos::CVector3& vec) const;
    argos::CVector3 ToVector3(const Vertex& vertex) const;
    argos::CRay3 ToVoronoiEdge(const Edge& edge) const;
    argos::CRay3 getRayBoundedToArena(const argos::CRay3 &ray) const;

    bool areVectorsEqual(const argos::CVector3 &a, const argos::CVector3 &b) const;
    bool areOnSameSide(const argos::CVector3 &a, const argos::CVector3 &b) const;
};


