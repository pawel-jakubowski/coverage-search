#pragma once

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class ShowRobotsIds : public CQTOpenGLUserFunctions {
public:
   ShowRobotsIds();
   virtual ~ShowRobotsIds() {}

   void Draw(CFootBotEntity& c_entity);
};
