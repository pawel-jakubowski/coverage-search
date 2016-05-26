#pragma once

namespace argos {
class CDynamics2DDifferentialSteeringControl;
class CDynamics2DGripper;
class CDynamics2DGrippable;
class CDynamics2DETargetModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include "target_entity.h"


namespace argos {

class CDynamics2DETargetModel : public CDynamics2DSingleBodyObjectModel {
public:
    CDynamics2DETargetModel(CDynamics2DEngine& engine, CTargetEntity& entity);
    virtual ~CDynamics2DETargetModel();
    virtual void Reset();
    virtual void UpdateFromEntityStatus();
};

}

