#pragma once

#include <core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <plugins/robots/generic/control_interface/ci_positioning_sensor.h>

namespace argos {

class Target : public CCI_Controller {
public:
    Target() = default;
    virtual ~Target() = default;

    virtual void Init(TConfigurationNode& configuration);
    virtual void ControlStep();
    virtual void Reset();
    virtual void Destroy() {}

private:
    CCI_RangeAndBearingActuator*  rabTx = nullptr;
    CCI_PositioningSensor* positioningSensor = nullptr;

    int getId() const;
};

}
