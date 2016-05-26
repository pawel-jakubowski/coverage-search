#include "target_entity.h"

#include "target_details.h"
#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>

namespace argos {

CTargetEntity::CTargetEntity() : CComposableEntity(nullptr) {}

CTargetEntity::CTargetEntity(const std::string& id,
                             const std::string& controllerId,
                             const CVector3& position,
                             const CQuaternion& orientation,
                             Real rabRange,
                             size_t rabDataSize)
    : CComposableEntity(nullptr, id) {
    try {
        embodiedEntity = new CEmbodiedEntity(this, bodyId, position, orientation);
        embodiedEntity->SetMovable(false);
        AddComponent(*embodiedEntity);

        ledEquippedEntity = new CLEDEquippedEntity(this, ledsId);
        AddComponent(*ledEquippedEntity);
        ledEquippedEntity->AddLEDRing(
            CVector3(0.0f, 0.0f, LED_RING_ELEVATION),
            LED_RING_RADIUS,
            LED_RING_START_ANGLE,
            8,
            embodiedEntity->GetOriginAnchor());

        rabEquippedEntity = new CRABEquippedEntity(this,
                                                   rabId,
                                                   rabDataSize,
                                                   rabRange,
                                                   embodiedEntity->GetOriginAnchor(),
                                                   *embodiedEntity,
                                                   CVector3(0.0f, 0.0f, RAB_ELEVATION));
        AddComponent(*rabEquippedEntity);

        controllableEntity = new CControllableEntity(this, controllableId);
        AddComponent(*controllableEntity);
        controllableEntity->SetController(controllerId);

        UpdateComponents();
    }
    catch (CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
    }
}

/****************************************/
/****************************************/

void CTargetEntity::Init(TConfigurationNode& t_tree) {
    try {
        CComposableEntity::Init(t_tree);

        embodiedEntity = new CEmbodiedEntity(this);
        AddComponent(*embodiedEntity);
        embodiedEntity->Init(GetNode(t_tree, "body"));

        ledEquippedEntity = new CLEDEquippedEntity(this, ledsId);
        AddComponent(*ledEquippedEntity);
        ledEquippedEntity->AddLEDRing(
           CVector3(0.0f, 0.0f, LED_RING_ELEVATION),
           LED_RING_RADIUS,
           LED_RING_START_ANGLE,
           8,
           embodiedEntity->GetOriginAnchor());

        Real fRange = 0.8f;
        GetNodeAttributeOrDefault(t_tree, "rab_range", fRange, fRange);
        UInt32 unDataSize = 2;
        GetNodeAttributeOrDefault(t_tree, "rab_data_size", unDataSize, unDataSize);
        rabEquippedEntity = new CRABEquippedEntity(this,
                                                  rabId,
                                                  unDataSize,
                                                  fRange,
                                                  embodiedEntity->GetOriginAnchor(),
                                                  *embodiedEntity,
                                                  CVector3(0.0f, 0.0f, RAB_ELEVATION));
        AddComponent(*rabEquippedEntity);

        controllableEntity = new CControllableEntity(this);
        AddComponent(*controllableEntity);
        controllableEntity->Init(GetNode(t_tree, "controller"));

        UpdateComponents();
    }
    catch (CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
    }
}

/****************************************/
/****************************************/

void CTargetEntity::Reset() {
    CComposableEntity::Reset();
    UpdateComponents();
}

/****************************************/
/****************************************/

void CTargetEntity::Destroy() {
    CComposableEntity::Destroy();
}

/****************************************/
/****************************************/

#define UPDATE(COMPONENT) if(COMPONENT->IsEnabled()) COMPONENT->Update();

void CTargetEntity::UpdateComponents() {
    UPDATE(rabEquippedEntity);
    UPDATE(ledEquippedEntity);
}

/****************************************/
/****************************************/

REGISTER_ENTITY(CTargetEntity,
                "target",
                "Pawel Jakubowski",
                "1.0",
                "The target robot.",
                "",
                "Under development"
   );

/****************************************/
/****************************************/

REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CTargetEntity);

/****************************************/
/****************************************/

}
