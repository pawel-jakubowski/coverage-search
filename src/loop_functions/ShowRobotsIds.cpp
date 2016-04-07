#include "ShowRobotsIds.h"

ShowRobotsIds::ShowRobotsIds() {
    RegisterUserFunction<ShowRobotsIds, CFootBotEntity>(&ShowRobotsIds::Draw);
}

void ShowRobotsIds::Draw(CFootBotEntity& c_entity) {
    glDisable(GL_LIGHTING);
    /* Disable face culling to be sure the text is visible from anywhere */
    glDisable(GL_CULL_FACE);
    /* Set the text color */
    CColor cColor(CColor::BLACK);
    glColor3ub(cColor.GetRed(), cColor.GetGreen(), cColor.GetBlue());
    /* The position of the text is expressed wrt the reference point of the footbot
     * For a foot-bot, the reference point is the center of its base.
     * See also the description in
     * $ argos3 -q foot-bot
     */
    GetOpenGLWidget().renderText(0.0, 0.0, 0.1,             // position
            c_entity.GetId().c_str()); // text

    /* Restore face culling */
    glEnable(GL_CULL_FACE);
    /* Restore lighting */
    glEnable(GL_LIGHTING);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(ShowRobotsIds, "show_id")
