#pragma once

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "CellularDecomposition.h"

class CellularDrawer : public argos::CQTOpenGLUserFunctions {
public:
    CellularDrawer();
    virtual ~CellularDrawer() {}
    void DrawInWorld() override;
private:
    const argos::UInt8 gridColor = 40;
    const argos::UInt8 gridFloorDiff = 255 - gridColor;
    CellularDecomposition& loopFnc;

    void drawCoverageGrid();
    void drawCoverageCell(const CoverageGrid::Cell& cell);

    void drawTaskCells();
    void drawCell(const TaskCell& cell, argos::Real liftOnZ);
};
