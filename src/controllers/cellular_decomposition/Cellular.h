#pragma once

#include <core/control_interface/ci_controller.h>
#include <core/utility/math/vector2.h>
#include <plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

#include <loop_functions/cellular_decomposition/CellularDecomposition.h>
#include <utils/task/TaskHandler.h>
#include <utils/task/BehaviorFactory.h>
#include <utils/task/behaviors/AvoidObstacleBehavior.h>


namespace argos {

class Cellular : public CCI_Controller, public TaskHandler {
    const Real angleEpsilon = .1;
public:
    Cellular();
    virtual ~Cellular() {}

    virtual void Init(TConfigurationNode& configuration) override;
    virtual void Reset() override;
    virtual void Destroy() override;
    virtual void ControlStep() override;

    virtual void update(Task newTask) override;
    virtual CVector2 getPosition() const override;
    virtual bool isCriticalPoint() const override;
    virtual bool isForwardConvexCP() const override;
    virtual bool isConcaveCP() const override;
    virtual bool isReadyToProceed() const override;

private:
    CCI_DifferentialSteeringActuator* wheelsEngine = nullptr;
    CCI_FootBotProximitySensor* proximitySensor = nullptr;
    CCI_PositioningSensor* positioningSensor = nullptr;
    CCI_LightSensor* lightSensor = nullptr;
    CCI_RangeAndBearingSensor* rabRx = nullptr;
    CCI_LEDsActuator* leds = nullptr;
    CCI_ColoredBlobPerspectiveCameraSensor* cameraLeft = nullptr;
    CCI_ColoredBlobPerspectiveCameraSensor* cameraRight = nullptr;
    CCI_ColoredBlobPerspectiveCameraSensor* cameraFront = nullptr;
    CCI_ColoredBlobPerspectiveCameraSensor* cameraBack = nullptr;

    std::shared_ptr<TaskManager> taskManager;
    CellularDecomposition& loopFnc;
    std::unique_ptr<BehaviorFactory> factory;
    std::shared_ptr<ControllerBehavior> behavior;
    std::shared_ptr<AvoidObstacleBehavior> avoidBehavior;

    bool criticalPointDetected = false;
    bool forwardConvexCPDetected = false;
    bool concaveCPDetected = false;
    bool readyToProceed = false;

    void logCurrentTask() const;
    CVector2 runBehavior();
    void detectTargets();
};

}
