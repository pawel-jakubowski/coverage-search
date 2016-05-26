#include "qtopengl_target.h"

#include "target_entity.h"
#include "target_details.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>


namespace argos {

CQTOpenGLTarget::CQTOpenGLTarget()
    : roundPartsVerticesNumber(40)
    , angleGapBetweenLeds(360.0f / 8.0f) {
    /* Reserve the needed display lists */
    displayListIndex = glGenLists(3);

    /* Assign indices for better referencing (later) */
    bodyDisplayListIndex = displayListIndex + 1;
    ledDisplayListIndex = displayListIndex + 2;

    /* Create the body display list */
    glNewList(bodyDisplayListIndex, GL_COMPILE);
    RenderBody();
    glEndList();

    /* Create the LED display list */
    glNewList(ledDisplayListIndex, GL_COMPILE);
    RenderLED();
    glEndList();
}

/****************************************/
/****************************************/

CQTOpenGLTarget::~CQTOpenGLTarget() {
    glDeleteLists(displayListIndex, 3);
}

/****************************************/
/****************************************/

void CQTOpenGLTarget::Draw(CTargetEntity& entity) {
    /* Place the body */
    glCallList(bodyDisplayListIndex);
    /* Place the LEDs */
    glPushMatrix();
    CLEDEquippedEntity& ledEquippedEntity = entity.GetLEDEquippedEntity();
    for (const auto& led : ledEquippedEntity.GetLEDs()) {
        const CColor& color = led->LED.GetColor();
        glRotatef(-angleGapBetweenLeds, 0.0f, 0.0f, 1.0f);
        SetLEDMaterial(color.GetRed(),
                       color.GetGreen(),
                       color.GetBlue());
        glCallList(ledDisplayListIndex);
    }
    glPopMatrix();
}

/****************************************/
/****************************************/

void CQTOpenGLTarget::SetBodyPlasticMaterial() {
    const GLfloat pfColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
    const GLfloat pfSpecular[] = {0.0f, 0.0f, 0.0f, 1.0f};
    const GLfloat pfShininess[] = {100.0f};
    const GLfloat pfEmission[] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
}

/****************************************/
/****************************************/

void CQTOpenGLTarget::SetRedPlasticMaterial() {
    const GLfloat pfColor[] = {1.0f, 0.0f, 0.0f, 1.0f};
    const GLfloat pfSpecular[] = {0.9f, 0.9f, 0.9f, 1.0f};
    const GLfloat pfShininess[] = {100.0f};
    const GLfloat pfEmission[] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
}

/****************************************/
/****************************************/

void CQTOpenGLTarget::SetCircuitBoardMaterial() {
    const GLfloat pfColor[] = {0.0f, 0.0f, 1.0f, 1.0f};
    const GLfloat pfSpecular[] = {0.5f, 0.5f, 1.0f, 1.0f};
    const GLfloat pfShininess[] = {10.0f};
    const GLfloat pfEmission[] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
}

/****************************************/
/****************************************/

void CQTOpenGLTarget::SetLEDMaterial(GLfloat red,
                                       GLfloat green,
                                       GLfloat blue) {
    const GLfloat fEmissionFactor = 10.0f;
    const GLfloat pfColor[] = {red, green, blue, 1.0f};
    const GLfloat pfSpecular[] = {0.0f, 0.0f, 0.0f, 1.0f};
    const GLfloat pfShininess[] = {0.0f};
    const GLfloat pfEmission[] = {red * fEmissionFactor, green * fEmissionFactor, blue * fEmissionFactor, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
}

/****************************************/
/****************************************/

void CQTOpenGLTarget::RenderBody() {
    /* Set material */
    SetBodyPlasticMaterial();
    CVector2 vertex(BODY_RADIUS, 0.0f);
    CRadians angle(CRadians::TWO_PI / roundPartsVerticesNumber);
    /* Top part */
    glBegin(GL_POLYGON);
    vertex.Set(LED_RING_INNER_RADIUS, 0.0f);
    glNormal3f(0.0f, 0.0f, 1.0f);
    for (GLuint i = 0; i <= roundPartsVerticesNumber; i++) {
        glVertex3f(vertex.GetX(), vertex.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
        vertex.Rotate(angle);
    }
    glEnd();
    /* Side surface */
    CVector2 normal(1.0f, 0.0f);
    vertex.Set(BODY_RADIUS, 0.0f);
    glBegin(GL_QUAD_STRIP);
    for (GLuint i = 0; i <= roundPartsVerticesNumber; i++) {
        glNormal3f(normal.GetX(), normal.GetY(), 0.0f);
        glVertex3f(vertex.GetX(), vertex.GetY(), BODY_ELEVATION + BODY_HEIGHT);
        glVertex3f(vertex.GetX(), vertex.GetY(), BODY_ELEVATION);
        vertex.Rotate(angle);
        normal.Rotate(angle);
    }
    glEnd();
    /* Bottom part */
    angle = -angle;
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, 0.0f, -1.0f);
    for (GLuint i = 0; i <= roundPartsVerticesNumber; i++) {
        glVertex3f(vertex.GetX(), vertex.GetY(), BODY_ELEVATION);
        vertex.Rotate(angle);
    }
    glEnd();
    /* Bulls eye */
    SetRedPlasticMaterial();
    angle = -angle;
    glBegin(GL_POLYGON);
    vertex.Set(0.45 * LED_RING_INNER_RADIUS, 0.0f);
    glNormal3f(0.0f, 0.0f, 1.0f);
    for (GLuint i = 0; i <= roundPartsVerticesNumber; i++) {
        glVertex3f(vertex.GetX(), vertex.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT + 0.001f);
        vertex.Rotate(angle);
    }
    glEnd();
}

/****************************************/
/****************************************/

void CQTOpenGLTarget::RenderLED() {
    /* Side surface */
    CVector2 vertex(BODY_RADIUS, 0.0f);
    CRadians angle(CRadians::TWO_PI / roundPartsVerticesNumber);
    CVector2 normal(1.0f, 0.0f);
    glBegin(GL_QUAD_STRIP);
    for (GLuint i = 0; i <= roundPartsVerticesNumber / 8; i++) {
        glNormal3f(normal.GetX(), normal.GetY(), 0.0f);
        glVertex3f(vertex.GetX(), vertex.GetY(), LED_RING_ELEVATION + LED_HEIGHT);
        glVertex3f(vertex.GetX(), vertex.GetY(), LED_RING_ELEVATION);
        vertex.Rotate(angle);
        normal.Rotate(angle);
    }
    glEnd();
    /* Top surface  */
    vertex.Set(BODY_RADIUS, 0.0f);
    CVector2 vertex2(LED_RING_INNER_RADIUS, 0.0f);
    glBegin(GL_QUAD_STRIP);
    glNormal3f(0.0f, 0.0f, 1.0f);
    for (GLuint i = 0; i <= roundPartsVerticesNumber / 8; i++) {
        glVertex3f(vertex2.GetX(), vertex2.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
        glVertex3f(vertex.GetX(), vertex.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
        vertex.Rotate(angle);
        vertex2.Rotate(angle);
    }
    glEnd();
}

/****************************************/
/****************************************/

class CQTOpenGLOperationDrawTargetNormal : public CQTOpenGLOperationDrawNormal {
public:
    void ApplyTo(CQTOpenGLWidget& visualization,
                 CTargetEntity& entity) {
        static CQTOpenGLTarget model;
        visualization.DrawRays(entity.GetControllableEntity());
        visualization.DrawEntity(entity.GetEmbodiedEntity());
        model.Draw(entity);
    }
};

class CQTOpenGLOperationDrawTargetSelected : public CQTOpenGLOperationDrawSelected {
public:
    void ApplyTo(CQTOpenGLWidget& visualization,
                 CTargetEntity& entity) {
        visualization.DrawBoundingBox(entity.GetEmbodiedEntity());
    }
};

REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawTargetNormal, CTargetEntity
);

REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawTargetSelected, CTargetEntity
);

/****************************************/
/****************************************/

}
