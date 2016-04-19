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
    const argos::CColor gridColor = argos::CColor::GRAY90;
    const argos::CColor voronoiVertexColor = argos::CColor::BLACK;
    const argos::Real vertexSize = 5.0f;
    MbfoLoopFunction& mbfo;

    void drawGrid();
    void drawVertices();
    void drawEdges();

    void drawVoronoi();

    void drawCell(const CoverageGrid::Cell &cell);
};


