#pragma once

namespace argos {
class CQTOpenGLTarget;
class CTargetEntity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

class CQTOpenGLTarget {
public:
    CQTOpenGLTarget();
    virtual ~CQTOpenGLTarget();
    virtual void Draw(CTargetEntity& entity);

protected:
    void SetBodyPlasticMaterial();
    void SetRedPlasticMaterial();
    void SetCircuitBoardMaterial();
    void SetLEDMaterial(GLfloat red, GLfloat green, GLfloat blue);

    void RenderBody();
    void RenderLED();

private:
    GLuint displayListIndex;
    GLuint bodyDisplayListIndex;
    GLuint ledDisplayListIndex;
    GLuint roundPartsVerticesNumber;
    GLfloat angleGapBetweenLeds;
};

}

