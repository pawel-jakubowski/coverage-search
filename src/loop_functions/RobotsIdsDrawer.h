#pragma once

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class RobotsIdsDrawer : public CQTOpenGLUserFunctions {
public:
   RobotsIdsDrawer();
   virtual ~RobotsIdsDrawer() {}

   void Draw(CFootBotEntity& c_entity);
};
