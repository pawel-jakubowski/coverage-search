#include "CalculateVoronoi.h"

using namespace argos;
using namespace boost::polygon;

CalculateVoronoi::Point CalculateVoronoi::ToPoint(const CVector3& vec) {
    Point point;
    /* Assuming that Vector have meter values */
    point.set(HORIZONTAL, static_cast<int>(vec.GetX()*scaleVectorToMilimeters));
    point.set(VERTICAL, static_cast<int>(vec.GetY()*scaleVectorToMilimeters));
    return point;
}

argos::CVector3 CalculateVoronoi::ToVector3(const voronoi_vertex<double>& point) {
    Real x = static_cast<Real>(point.x()) / scaleVectorToMilimeters;
    Real y = static_cast<Real>(point.y()) / scaleVectorToMilimeters;
    return CVector3(x, y, 0.1f);
}

CalculateVoronoi::VoronoiEdge CalculateVoronoi::ToVoronoiEdge(
        const boost::polygon::voronoi_edge<double>& edge) {
    assert(edge.vertex0() != nullptr);
    assert(edge.vertex1() != nullptr);

    VoronoiEdge voronoiEdge;
    voronoiEdge.begin = ToVector3(*edge.vertex0());
    voronoiEdge.end = ToVector3(*edge.vertex1());
    return voronoiEdge;
}

void CalculateVoronoi::Init(TConfigurationNode& t_tree) {
    auto& entities = GetSpace().GetEntitiesByType("foot-bot");
    LOG << "Size: " << entities.size() << std::endl;

    for (const auto& entity : entities) {
        auto& footbot = *(any_cast<CFootBotEntity*>(entity.second));
        auto position =
            ToPoint(footbot.GetEmbodiedEntity().GetOriginAnchor().Position);
        points.push_back(position);
        LOG << "(" << position.x() << ", " << position.y() << ")" "\n";
    }

    voronoi_diagram<Real> voronoiDiagram;
    construct_voronoi(points.begin(), points.end(), &voronoiDiagram);

    parseVertices(voronoiDiagram);

    for (auto& cell : voronoiDiagram.cells()) {
        const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();
        assert(edge != nullptr);
        do {
            if (edge->is_primary() && edge->is_finite())
                edges.push_back(ToVoronoiEdge(*edge));
            edge = edge->next();
        } while (edge != cell.incident_edge());
    }
}

void CalculateVoronoi::parseVertices(const voronoi_diagram<Real> &voronoiDiagram) {
    for (auto& vertex : voronoiDiagram.vertices())
        vertices.push_back(ToVector3(vertex));

    LOG << "\n" "Voronoi vetrices:" "\n";
    for (auto& vertex : vertices) {
        LOG << "(" << vertex.GetX() << ", " << vertex.GetY() << ")" "\n";
    }
}

std::vector<argos::CVector3> CalculateVoronoi::getVertices() {
    return vertices;
}

std::vector<CalculateVoronoi::VoronoiEdge> CalculateVoronoi::getEdges() {
    return edges;
}

REGISTER_LOOP_FUNCTIONS(CalculateVoronoi, "calculate_voronoi")
