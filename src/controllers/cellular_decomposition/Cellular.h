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
    virtual CVector2 getPosition() override;
    virtual bool isCriticalPoint() override;
    virtual bool isReadyToProceed() override;

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

    CellularDecomposition& loopFnc;
    std::unique_ptr<BehaviorFactory> factory;
    std::shared_ptr<ControllerBehavior> behavior;

    bool criticalPointDetected = false;
    bool readyToProceed = false;

    void logCurrentTask() const;

//    enum class Direction { Left, Right };
//    CDegrees lastRotation;
//    Real velocity;
//    Real rotationSpeed;
//    Real minDistanceFromObstacle;
//    CDegrees lastControl;
//    void rotateForAnAngle(const CDegrees &angle);
//    CDegrees getOrientationOnXY();
//    Direction getRotationDirection(const CDegrees& obstacleAngle);
//    CDegrees getRotationAngle() const;
//    void rotate(Direction rotationDirection);
//    void move(const CDegrees& rotationAngle);
//    CVector2 getAccumulatedVector(const CCI_FootBotProximitySensor::TReadings& readings, Real threshold) const;
//    CDegrees getAngleBetweenPoints(const CVector2& a, const CVector2& b) const;
//    void moveToPoint(const CVector2& point);
//
//    void stopWheels() const;
//
//    CDegrees myPositionToPointAngle(const CVector2& point);
//    CDegrees getControl(const CDegrees& rotationAngle) const;
//
//    CDegrees getfellowAngle(const CColor& color) const;
//    bool isFellowVisible(const CColor& color) const;
//
//    CCI_FootBotProximitySensor::TReadings getLeftProximityReadings() const;
//    CCI_FootBotProximitySensor::TReadings getBackProximityReadings() const;
//    CCI_FootBotProximitySensor::TReadings getRightProximityReadings() const;
//    CCI_FootBotProximitySensor::TReadings getFrontProximityReadings() const;
//
//    void explorerMove(const CColor& explorerColor, const CColor& fellowColor, bool (* isDesiredAngle)(const CDegrees&));
//    CColor getMyColor() const;
//    CColor getFellowColor() const;
};

}
