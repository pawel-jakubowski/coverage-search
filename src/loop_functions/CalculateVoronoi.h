#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <boost/polygon/point_data.hpp>
#include <boost/polygon/voronoi.hpp>

class CalculateVoronoi : public argos::CLoopFunctions {
public:
    struct VoronoiEdge {
        argos::CVector3 begin;
        argos::CVector3 end;
    };

    CalculateVoronoi() {}
    virtual ~CalculateVoronoi() {}
    virtual void Init(argos::TConfigurationNode& t_tree) override;

    std::vector<argos::CVector3> getVertices();
    std::vector<VoronoiEdge> getEdges();

private:
    using Point = boost::polygon::point_data<int>;

    const int scaleVectorToMilimeters = 100000;
    std::vector<Point> points;
    std::vector<argos::CVector3> vertices;
    std::vector<VoronoiEdge> edges;

    Point ToPoint(const argos::CVector3& vec);
    argos::CVector3 ToVector3(const boost::polygon::voronoi_vertex<double>& vertex);
    VoronoiEdge ToVoronoiEdge(const boost::polygon::voronoi_edge<double>& edge);

    void parseVertices(const boost::polygon::voronoi_diagram<argos::Real> &voronoiDiagram);
};


