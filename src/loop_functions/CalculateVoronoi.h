#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <boost/polygon/point_data.hpp>
#include <boost/polygon/voronoi.hpp>


class CalculateVoronoi : public argos::CLoopFunctions {
public:
    CalculateVoronoi() {}
    virtual ~CalculateVoronoi() {}
    virtual void Init(argos::TConfigurationNode& t_tree) override;

    void update();
    std::vector<argos::CVector3> getVertices();
    std::vector<argos::CRay3> getEdges();

private:
    using CoordinateType = argos::Real;
    using Point = boost::polygon::point_data<CoordinateType>;
    using VoronoiDiagram = boost::polygon::voronoi_diagram<CoordinateType>;
    using Vertex = boost::polygon::voronoi_vertex<CoordinateType>;
    using Edge = boost::polygon::voronoi_edge<CoordinateType>;

    const int scaleVectorToMilimeters = 100000;
    const argos::Real diagramLiftOnZ = 0.1f;
    argos::CRange<argos::CVector3> arenaLimits;
    std::vector<Point> points;
    std::vector<argos::CVector3> vertices;
    std::vector<argos::CRay3> edges;

    void reset();
    void updateVoronoiDiagram();
    void updateRobotsPositions(const argos::CSpace::TMapPerType& entities);
    void updateVertices(const VoronoiDiagram& voronoiDiagram);
    void updateEdges(const VoronoiDiagram& voronoiDiagram);

    Point ToPoint(const argos::CVector3& vec) const;
    argos::CVector3 ToVector3(const Vertex& vertex) const;
    argos::CRay3 ToVoronoiEdge(const Edge& edge) const;
    argos::CRay3 getRayBoundedToArena(const argos::CRay3 &ray) const;
};


