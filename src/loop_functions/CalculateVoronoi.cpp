#include "CalculateVoronoi.h"

using namespace argos;
using namespace boost::polygon;

CalculateVoronoi::Point CalculateVoronoi::ToPoint(const CVector3& vec) const {
    Point point;
    /* Assuming that Vector have meter values */
    point.set(HORIZONTAL, vec.GetX()*scaleVectorToMilimeters);
    point.set(VERTICAL, vec.GetY()*scaleVectorToMilimeters);
    return point;
}

argos::CVector3 CalculateVoronoi::ToVector3(const Vertex& point) const {
    Real x = point.x() / scaleVectorToMilimeters;
    Real y = point.y() / scaleVectorToMilimeters;
    return CVector3(x, y, diagramLiftOnZ);
}

CRay3 CalculateVoronoi::ToVoronoiEdge(const Edge& edge) const {
    CRay3 voronoiEdge;
    if (edge.is_finite()) {
        voronoiEdge.SetStart(ToVector3(*edge.vertex0()));
        voronoiEdge.SetEnd(ToVector3(*edge.vertex1()));
    }
    else {
        const auto& cell1 = *edge.cell();
        const auto& cell2 = *edge.twin()->cell();
        Point origin, direction;

        Point p1 = points.at(cell1.source_index());
        Point p2 = points.at(cell2.source_index());
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
            voronoiEdge.SetEnd(end);
        } else {
            voronoiEdge.SetEnd(ToVector3(*edge.vertex1()));
        }
    }
    return getRayBoundedToArena(voronoiEdge);
}

// Liang-Barsky function by Daniel White @ http://www.skytopia.com/project/articles/compsci/clipping.html
// This function inputs 8 numbers, and outputs 4 new numbers (plus a boolean value to say whether the clipped line is drawn at all).
//
bool LiangBarsky (double edgeLeft, double edgeRight, double edgeBottom, double edgeTop,   // Define the x/y clipping values for the border.
                  double x0src, double y0src, double x1src, double y1src,                 // Define the start and end points of the line.
                  double &x0clip, double &y0clip, double &x1clip, double &y1clip)         // The output values, so declare these outside.
{

    double t0 = 0.0;    double t1 = 1.0;
    double xdelta = x1src-x0src;
    double ydelta = y1src-y0src;
    double p,q,r;

    for(int edge=0; edge<4; edge++) {   // Traverse through left, right, bottom, top edges.
        if (edge==0) {  p = -xdelta;    q = -(edgeLeft-x0src);  }
        if (edge==1) {  p = xdelta;     q =  (edgeRight-x0src); }
        if (edge==2) {  p = -ydelta;    q = -(edgeBottom-y0src);}
        if (edge==3) {  p = ydelta;     q =  (edgeTop-y0src);   }
        r = q/p;
        if(p==0 && q<0) return false;   // Don't draw line at all. (parallel line outside)

        if(p<0) {
            if(r>t1) return false;         // Don't draw line at all.
            else if(r>t0) t0=r;            // Line is clipped!
        } else if(p>0) {
            if(r<t0) return false;      // Don't draw line at all.
            else if(r<t1) t1=r;         // Line is clipped!
        }
    }

    x0clip = x0src + t0*xdelta;
    y0clip = y0src + t0*ydelta;
    x1clip = x0src + t1*xdelta;
    y1clip = y0src + t1*ydelta;

    return true;        // (clipped) line is drawn
}

CRay3 CalculateVoronoi::getRayBoundedToArena(const CRay3 &ray) const {
//    LOG << "Get edge: start=" << ray.GetStart()
//        << ", end=" << ray.GetEnd()
//        << ", length=" << ray.GetLength()
//        << "\n";
    Real startX, startY, endX, endY;
    LiangBarsky(
            arenaLimits.GetMin().GetX(),
            arenaLimits.GetMax().GetX(),
            arenaLimits.GetMin().GetY(),
            arenaLimits.GetMax().GetY(),
            ray.GetStart().GetX(), ray.GetStart().GetY(),
            ray.GetEnd().GetX(), ray.GetEnd().GetY(),
            startX, startY, endX, endY);
    CRay3 boundedRay(CVector3(startX, startY, diagramLiftOnZ),
                     CVector3(endX, endY, diagramLiftOnZ));\
//    LOG << "Add edge: start=" << boundedRay.GetStart()
//        << ", end=" << boundedRay.GetEnd()
//        << ", length=" << boundedRay.GetLength()
//        << "\n";

    return boundedRay;
}

void CalculateVoronoi::Init(TConfigurationNode& t_tree) {
    arenaLimits = GetSpace().GetArenaLimits();
    update();
}

void CalculateVoronoi::update() {
    reset();
    auto& entities = this->GetSpace().GetEntitiesByType("foot-bot");
    updateRobotsPositions(entities);
    updateVoronoiDiagram();
}

void CalculateVoronoi::reset() {
    this->points.clear();
    this->vertices.clear();
    this->edges.clear();
}

void CalculateVoronoi::updateRobotsPositions(const CSpace::TMapPerType &entities) {
    for (const auto& entity : entities) {
        auto& footbot = *(any_cast<CFootBotEntity*>(entity.second));
        auto position =
            ToPoint(footbot.GetEmbodiedEntity().GetOriginAnchor().Position);
        points.push_back(position);
    }
}

void CalculateVoronoi::updateVoronoiDiagram() {
    voronoi_diagram<Real> voronoiDiagram;
    construct_voronoi(points.begin(), points.end(), &voronoiDiagram);
    updateVertices(voronoiDiagram);
    updateEdges(voronoiDiagram);
}

void CalculateVoronoi::updateVertices(const voronoi_diagram<Real> &voronoiDiagram) {
    CVector3 vectorVertex;
    for (auto& vertex : voronoiDiagram.vertices()) {
        vectorVertex = ToVector3(vertex);
        if (arenaLimits.WithinMinBoundIncludedMaxBoundIncluded(vectorVertex))
            vertices.emplace_back(vectorVertex);
    }
}

void CalculateVoronoi::updateEdges(const voronoi_diagram<Real> &voronoiDiagram) {
    for (auto& cell : voronoiDiagram.cells()) {
        const voronoi_diagram<Real>::edge_type* edge = cell.incident_edge();
        assert(edge != nullptr);
        assert(edge->is_linear()); // For points all edges should be linear
        do {
            if (edge->is_primary())
                edges.push_back(ToVoronoiEdge(*edge));
            edge = edge->next();
        } while (edge != cell.incident_edge());
    }
}

std::vector<argos::CVector3> CalculateVoronoi::getVertices() {
    return vertices;
}

std::vector<CRay3> CalculateVoronoi::getEdges() {
    return edges;
}

REGISTER_LOOP_FUNCTIONS(CalculateVoronoi, "calculate_voronoi")
