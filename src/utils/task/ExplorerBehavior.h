#pragma once

#include "ControllerBehavior.h"
#include <core/utility/math/vector2.h>

class ExplorerBehavior : public ControllerBehavior {
public:
    ExplorerBehavior(Sensors s, Actuators a,
                     argos::CColor myColor,
                     argos::CColor fellowColor,
                     argos::Real frontThreshold  = 0.05f,
                     argos::Real frontAngleEpsilon = 2);
    virtual ~ExplorerBehavior() = default;

    virtual void proceed() override;

    virtual bool isCriticalPoint() const override;
    virtual bool isConvexCP() const override;
    virtual bool isConcaveCP() const override;

protected:
    static constexpr argos::Real angleEpsilon = .1;

    argos::CColor myColor;
    argos::CColor fellowColor;
    argos::Real frontThreshold;
    argos::Real frontAngleEpsilon;

    virtual argos::CDegrees getRotationAngle() const = 0;

    bool isFellowVisible() const;
    argos::CDegrees getfellowAngle() const;
    argos::CVector2 getAccumulatedVector(const argos::CCI_FootBotProximitySensor::TReadings& readings,
                                         argos::Real threshold) const;
    argos::CCI_FootBotProximitySensor::TReadings getFrontProximityReadings() const;

private:
    const argos::CDegrees leftCameraOffset = argos::CDegrees(135);
    const argos::CDegrees frontCameraOffset = argos::CDegrees(45);
    const argos::CDegrees rightCameraOffset = argos::CDegrees(-45);
    const argos::CDegrees backCameraOffset = argos::CDegrees(-135);
};


