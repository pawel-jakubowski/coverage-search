/**
 * File based on dynamics2d_epuck_model.cpp
 */

#include "dynamics2d_target_model.h"

#include "target_details.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_gripping.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>


namespace argos {


CDynamics2DETargetModel::CDynamics2DETargetModel(CDynamics2DEngine& engine, CTargetEntity& entity)
    : CDynamics2DSingleBodyObjectModel(engine, entity) {
    /* Create a static body */
    cpBody* ptBody = cpBodyNewStatic();
    const CVector3& position = GetEmbodiedEntity().GetOriginAnchor().Position;
    ptBody->p = cpv(position.GetX(), position.GetY());
    CRadians xAngle, yAngle, zAngle;
    GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(zAngle, yAngle, xAngle);
    cpBodySetAngle(ptBody, zAngle.GetValue());
    /* Create the shape */
    cpShape* ptShape =
        cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpCircleShapeNew(ptBody,
                                         BODY_RADIUS,
                                         cpvzero));
    ptShape->e = 0.0; // No elasticity
    ptShape->u = 0.1; // Little contact friction to help sliding away
    /* This shape is normal (not grippable, not gripper) */
    ptShape->collision_type = CDynamics2DEngine::SHAPE_NORMAL;
    /* Set the body so that the default methods work as expected */
    SetBody(ptBody, BODY_HEIGHT);
}

CDynamics2DETargetModel::~CDynamics2DETargetModel() {}

void CDynamics2DETargetModel::Reset() {
    CDynamics2DSingleBodyObjectModel::Reset();
}

void CDynamics2DETargetModel::UpdateFromEntityStatus() {}

REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CTargetEntity, CDynamics2DETargetModel);

}
