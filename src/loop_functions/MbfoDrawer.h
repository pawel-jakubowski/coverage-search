#pragma once

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "MbfoLoopFunction.h"

class MbfoDrawer : public argos::CQTOpenGLUserFunctions {
public:
    MbfoDrawer();
    virtual ~MbfoDrawer() {}
    void DrawInWorld() override;
private:
    const argos::UInt8 gridColor = 40;
    const argos::UInt8 gridFloorDiff = 255 - gridColor;
    const argos::CColor voronoiVertexColor = argos::CColor::BLACK;
    const argos::Real vertexSize = 5.0f;
    MbfoLoopFunction& mbfo;

    void drawGrid();
    void drawVoronoi();

    void drawCell(const CoverageGrid::Cell &cell);

    void drawVertex(argos::CRay3& edge);
    void drawEdge(const argos::CRay3& edge);

    void drawCellId(const VoronoiDiagram::Cell& cell);
};
