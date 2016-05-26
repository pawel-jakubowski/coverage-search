#pragma once

namespace argos {
class CControllableEntity;
class CEmbodiedEntity;
class CTargetEntity;
class CLEDEquippedEntity;
class CRABEquippedEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

class CTargetEntity : public CComposableEntity {
    const std::string name = "target";
public:
    ENABLE_VTABLE();

public:
    CTargetEntity();
    CTargetEntity(const std::string& id,
                  const std::string& controllerId,
                  const CVector3& position = CVector3(),
                  const CQuaternion& orientation = CQuaternion(),
                  Real rabRange = 0.8f,
                  size_t rabDataSize = 1);

    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual void Destroy();

    virtual void UpdateComponents();

    inline CControllableEntity& GetControllableEntity() { return *controllableEntity; }
    inline CEmbodiedEntity& GetEmbodiedEntity() { return *embodiedEntity; }
    inline CLEDEquippedEntity& GetLEDEquippedEntity() { return *ledEquippedEntity; }
    inline CRABEquippedEntity& GetRABEquippedEntity() { return *rabEquippedEntity; }
    virtual std::string GetTypeDescription() const { return name; }

private:
    void SetLEDPosition();

private:
    CControllableEntity* controllableEntity = nullptr;
    CEmbodiedEntity* embodiedEntity = nullptr;
    CLEDEquippedEntity* ledEquippedEntity = nullptr;
    CRABEquippedEntity* rabEquippedEntity = nullptr;

    const std::string bodyId = "body";
    const std::string ledsId = "leds";
    const std::string rabId = "rab";
    const std::string controllableId = "controller";
};

}

