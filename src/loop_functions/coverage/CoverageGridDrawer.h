#pragma once

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include "CoverageCalculator.h"

class CoverageGridDrawer : public argos::CQTOpenGLUserFunctions {
public:
    CoverageGridDrawer();
    virtual ~CoverageGridDrawer() {}
    void DrawInWorld() override;
private:
    CoverageCalculator& coverage;
};


