#pragma once

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "CalculateVoronoi.h"

class VoronoiDrawer : public argos::CQTOpenGLUserFunctions {
public:
    VoronoiDrawer();
    virtual ~VoronoiDrawer() {}
    void DrawInWorld() override;
private:
    const argos::CColor vertexColor = argos::CColor::BLACK;
    const argos::Real vertexSize = 5.0f;
    CalculateVoronoi& voronoi;

    void drawVertices();

    void drawEdges();
};


