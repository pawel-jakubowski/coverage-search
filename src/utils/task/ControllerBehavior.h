#pragma once

#include <plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>

#include <plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>


struct Sensors {
    argos::CCI_PositioningSensor& position;
    argos::CCI_FootBotProximitySensor& proximity;
    struct Cameras {
        argos::CCI_ColoredBlobPerspectiveCameraSensor& left;
        argos::CCI_ColoredBlobPerspectiveCameraSensor& right;
        argos::CCI_ColoredBlobPerspectiveCameraSensor& front;
        argos::CCI_ColoredBlobPerspectiveCameraSensor& back;
    } cameras;
};

struct Actuators {
    argos::CCI_DifferentialSteeringActuator& wheels;
    argos::CCI_LEDsActuator& leds;
};


class ControllerBehavior {
public:
    ControllerBehavior(Sensors s, Actuators a);
    virtual ~ControllerBehavior() = default;
    virtual void proceed() = 0;
    virtual void prepare() = 0;
    virtual void moveToBegin(const argos::CVector2& beginning);
    virtual void stop();

    virtual bool isReadyToProceed() const { return true; }
    virtual bool isConcaveCP() const { return false; }
    virtual bool isCriticalPoint() const { return false; }
    virtual bool isConvexCP() const { return false; }
protected:
    enum class Direction { Left, Right };

    Sensors sensors;
    Actuators actuators;
    argos::CDegrees lastRotation;

    void move(const argos::CDegrees& rotationAngle);
    void rotateForAnAngle(const argos::CDegrees& angle);
    void rotate(Direction rotationDirection);
    argos::CDegrees getControl(const argos::CDegrees& rotationAngle, argos::Real KP = 0.5, argos::Real KD = 0.25) const;
    Direction getRotationDirection(const argos::CDegrees& obstacleAngle) const;

    argos::CDegrees myPositionToPointAngle(const argos::CVector2& point) const;
    argos::CDegrees getAngleBetweenPoints(const argos::CVector2& a, const argos::CVector2& b) const;
    argos::CDegrees getOrientationOnXY() const;

private:
    argos::Real rotationSpeed;
    argos::CDegrees lastControl;
};


