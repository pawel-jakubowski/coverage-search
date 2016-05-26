#include "Target.h"
#include <assert.h>

namespace argos {

void Target::Init(TConfigurationNode& configuration) {
    rabTx = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    positioningSensor = GetSensor<CCI_PositioningSensor>("positioning");
    assert(rabTx != nullptr);
    assert(positioningSensor != nullptr);
}

void Target::ControlStep() {
    CByteArray data;
    data << getId();
    data << positioningSensor->GetReading().Position.GetX();
    data << positioningSensor->GetReading().Position.GetY();
    data << positioningSensor->GetReading().Position.GetZ();
    rabTx->SetData(data);
}

int Target::getId() const {
    auto strId = GetId();
    size_t lastNotNumberCharacterIndex = strId.find_last_not_of("0123456789");

    int id;
    std::stringstream stream;
    stream << strId.substr(lastNotNumberCharacterIndex + 1);
    stream >> id;
    return id + 1;
}

void Target::Reset() {
    rabTx->ClearData();
}

}

REGISTER_CONTROLLER(Target, "target_controller")
