#pragma once

#include <argos3/plugins/robots/generic/simulator/colored_blob_perspective_camera_default_sensor.h>

namespace argos {

class CustomPerspectiveCameraLeft : public CColoredBlobPerspectiveCameraDefaultSensor {
public:
    CustomPerspectiveCameraLeft() = default;
    virtual ~CustomPerspectiveCameraLeft() = default;

    virtual void SetRobot(CComposableEntity& c_entity);
};

class CustomPerspectiveCameraRight : public CColoredBlobPerspectiveCameraDefaultSensor {
public:
    CustomPerspectiveCameraRight() = default;
    virtual ~CustomPerspectiveCameraRight() = default;

    virtual void SetRobot(CComposableEntity& c_entity);
};

class CustomPerspectiveCameraFront : public CColoredBlobPerspectiveCameraDefaultSensor {
public:
    CustomPerspectiveCameraFront() = default;
    virtual ~CustomPerspectiveCameraFront() = default;

    virtual void SetRobot(CComposableEntity& c_entity);
};

class CustomPerspectiveCameraBack : public CColoredBlobPerspectiveCameraDefaultSensor {
public:
    CustomPerspectiveCameraBack() = default;
    virtual ~CustomPerspectiveCameraBack() = default;

    virtual void SetRobot(CComposableEntity& c_entity);
};

}