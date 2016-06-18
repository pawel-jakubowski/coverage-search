#include <argos3/core/utility/logging/argos_log.h>
#include "Cellular.h"

using namespace std;
using namespace argos;


/* Footbot dimensions */
static const Real INTERWHEEL_DISTANCE        = 0.14f;
static const Real HALF_INTERWHEEL_DISTANCE   = INTERWHEEL_DISTANCE * 0.5f;
static const Real HALF_INTERWHEEL_DISTANCE_IN_CM = HALF_INTERWHEEL_DISTANCE * 100;
static const Real PROXIMITY_SENSOR_RING_RANGE = 0.1f;

/* Simulator parameters */
static const Real TICKS_PER_SEC = 10;


Cellular::Cellular()
    : loopFnc(dynamic_cast<CellularDecomposition&>(CSimulator::GetInstance().GetLoopFunctions()))
{}

void Cellular::Init(TConfigurationNode& configuration) {
    wheelsEngine = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    proximitySensor = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    positioningSensor = GetSensor<CCI_PositioningSensor>("positioning");
    lightSensor = GetSensor<CCI_LightSensor>("light");
    rabRx = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    leds = GetActuator<CCI_LEDsActuator>("leds");
    cameraLeft = GetSensor<CCI_ColoredBlobPerspectiveCameraSensor>("perspective_camera_left");
    cameraRight = GetSensor<CCI_ColoredBlobPerspectiveCameraSensor>("perspective_camera_right");

    GetNodeAttributeOrDefault(configuration, "velocity", velocity, velocity);
    GetNodeAttributeOrDefault(configuration, "min_distance", minDistanceFromObstacle,
                              minDistanceFromObstacle);

    assert(wheelsEngine != nullptr);
    assert(proximitySensor != nullptr);
    assert(positioningSensor != nullptr);
    assert(lightSensor != nullptr);
    assert(rabRx != nullptr);
    assert(leds != nullptr);
    assert(cameraLeft != nullptr);
    assert(cameraRight != nullptr);

    cameraLeft->Enable();
    cameraRight->Enable();

    LOG << "Register me : " << this << endl;
    loopFnc.registerToTaskManager(*this);
}

void Cellular::Reset() {
    currentTask = Task();
    lastControl = lastRotation = CDegrees();
}

void Cellular::Destroy() {
    LOG << "Unregister me : " << this << endl;
    loopFnc.unregisterFromTaskManager(*this);
}

CVector2 Cellular::getPostion() {
    CVector2 position;
    positioningSensor->GetReading().Position.ProjectOntoXY(position);
    return position;
}

void Cellular::ControlStep() {
//    CDegrees rotationAngle = getRotationAngle();
//    move(rotationAngle);

    LOG << "[" << GetId() << "]: ";
    logCurrentTask();

    if (currentTask.behavior == Task::Behavior::FollowLeftBoundary) {
        leds->SetAllColors(CColor::GREEN);
        CDegrees fellowAngle;
        CDegrees fellowDesiredAngle(-90);
        CDegrees cameraOffset(-45);
        size_t addedAnglesCounter = 0;
        for (auto& blob : cameraRight->GetReadings().BlobList)
            if (blob->Color == CColor::RED) {
                fellowAngle += cameraOffset - CDegrees((blob->X / 10));
                addedAnglesCounter++;
            }
        fellowAngle /= addedAnglesCounter;
        LOG << "Fellow at angle " << fellowAngle << "\n";

        stopWheels();
        if (currentTask.status == Task::Status::MoveToBegin)
            moveToPoint(currentTask.begin);
        else if (currentTask.status == Task::Status::MoveToEnd) {
//            auto angleToPoint = myPositionToPointAngle(currentTask.end);
//            auto angleDiff = getControl(angleToPoint);
//            if (angleDiff.GetAbsoluteValue() > angleEpsilon) {
//                rotateForAnAngle(angleDiff);
//                lastRotation = angleDiff;
//            }
//            else if (fellowAngle.GetValue() != 0 && fellowAngle >= fellowDesiredAngle)
//                moveToPoint(currentTask.end);
            CVector2 rotationVector = CVector2(1, ToRadians(getRotationAngle())).Normalize();

            Real inertiaCoef = 1.5;
            CVector2 inertiaVector = inertiaCoef * CVector2(1, ToRadians(lastControl)).Normalize();
            LOG << "Inertia angle " << ToDegrees(inertiaVector.Angle()) << "\n";

            CVector2 controlVector = (rotationVector + inertiaVector);
            LOG << "Control angle " << ToDegrees(controlVector.Angle()) << "\n";

            move(ToDegrees(controlVector.Angle()));
        }

    } else if (currentTask.behavior == Task::Behavior::FollowRightBoundary) {
        leds->SetAllColors(CColor::RED);
        CDegrees fellowAngle;
        CDegrees fellowDesiredAngle(90);
        CDegrees cameraOffset(135);
        size_t addedAnglesCounter = 0;
        for (auto& blob : cameraLeft->GetReadings().BlobList)
            if (blob->Color == CColor::GREEN) {
                fellowAngle += cameraOffset - CDegrees((blob->X / 10));
                addedAnglesCounter++;
            }
        fellowAngle /= addedAnglesCounter;
        LOG << "Fellow at angle " << fellowAngle << "\n";
        stopWheels();
        if (currentTask.status == Task::Status::MoveToBegin)
            moveToPoint(currentTask.begin);
        else if (currentTask.status == Task::Status::MoveToEnd) {
//            auto angleToPoint = myPositionToPointAngle(currentTask.end);
//            auto angleDiff = getControl(angleToPoint);
//            if (angleDiff.GetAbsoluteValue() > angleEpsilon) {
//                rotateForAnAngle(angleDiff);
//                lastRotation = angleDiff;
//            }
//            else if (fellowAngle.GetValue() != 0 && fellowAngle <= fellowDesiredAngle)
//                moveToPoint(currentTask.end);
            CVector2 rotationVector = CVector2(1, ToRadians(getRotationAngle())).Normalize();

//            Real inertiaCoef = 1.5;
//            CVector2 inertiaVector = inertiaCoef * CVector2(1, ToRadians(lastControl)).Normalize();
//            LOG << "Inertia angle " << ToDegrees(inertiaVector.Angle()) << "\n";

            CVector2 controlVector = rotationVector;
            LOG << "Control angle " << ToDegrees(controlVector.Angle()) << "\n";

            move(ToDegrees(controlVector.Angle()));
        }
    }

    LOG.Flush();
}

void Cellular::stopWheels() const { wheelsEngine->SetLinearVelocity(0, 0); }

void Cellular::logCurrentTask() const {
    switch(currentTask.behavior) {
        case Task::Behavior::Idle:
            LOG << "IDLE";
            break;
        case Task::Behavior::Sweep:
            LOG << "SWEEP";
            break;
        case Task::Behavior::FollowLeftBoundary:
            LOG << "FOLLOW LEFT";
            break;
        case Task::Behavior::FollowRightBoundary:
            LOG << "FOLLOW RIGHT";
            break;
    }
    LOG << " -> ";
    switch(currentTask.status) {
        case Task::Status::Wait:
            LOG << "WAIT";
            break;
        case Task::Status::MoveToBegin:
            LOG << "MOVE TO BEGIN";
            break;
        case Task::Status::MoveToEnd:
            LOG << "MOVE TO END";
            break;
    }
    LOG << "\n";
}

void Cellular::moveToPoint(const CVector2& point) {
    CDegrees rotationAngle = myPositionToPointAngle(point);
    move(rotationAngle);
}

CDegrees Cellular::myPositionToPointAngle(const CVector2& point) {
    CVector2 currentPoint;
    positioningSensor->GetReading().Position.ProjectOntoXY(currentPoint);
    CDegrees angle = getAngleBetweenPoints(currentPoint, point);
    CDegrees robotsOrientation = getOrientationOnXY();
    auto rotationAngle = (robotsOrientation - angle).SignedNormalize();
    return rotationAngle;
}

CDegrees Cellular::getAngleBetweenPoints(const CVector2& a, const CVector2& b) const {
    auto angle = ToDegrees(ATan2(b.GetY() - a.GetY(), b.GetX() - a.GetX()));
    return angle;
}

CDegrees Cellular::getRotationAngle() const {
    using TReadings = CCI_FootBotProximitySensor::TReadings;
    const auto& readings = proximitySensor->GetReadings();

    TReadings leftReadings(readings.begin() + 4, readings.begin() + 8);
    TReadings backReadings(readings.begin() + 10, readings.begin() + 14);
    TReadings rightReadings(readings.begin() + 16, readings.begin() + 20);
    TReadings frontReadings(readings.begin() + 22, readings.end());
    frontReadings.insert(frontReadings.end(), readings.begin(), readings.begin() + 2);

    CDegrees sideAngle;
    CVector2 accumulator;
    bool onlyFront = true;
    Real frontThreshold = 0.13;
    Real sideThreshold = 0.1;

    if (currentTask.behavior == Task::Behavior::FollowLeftBoundary) {
        sideAngle = CDegrees(90);
        accumulator = getAccumulatedVector(frontReadings, frontThreshold);
        if (accumulator.SquareLength() == 0) {
            onlyFront = false;
            LOG << "Only left proximity" << "\n";
            accumulator = getAccumulatedVector(leftReadings, sideThreshold);
        }
    }
    else if (currentTask.behavior == Task::Behavior::FollowRightBoundary) {
        sideAngle = CDegrees(-90);
        accumulator = getAccumulatedVector(frontReadings, frontThreshold);
        if (accumulator.SquareLength() == 0) {
            onlyFront = false;
            LOG << "Only right proximity" << "\n";
            accumulator = getAccumulatedVector(rightReadings, sideThreshold);
        }
    }

    CDegrees rotationAngle;
    if (!onlyFront)
        rotationAngle = lastRotation;
    if (accumulator.SquareLength() > 0) {
        rotationAngle = (sideAngle - ToDegrees(accumulator.Angle())).SignedNormalize();
    }
    LOG << "Rotation angle " << rotationAngle << "\n";
    return rotationAngle;
}

CVector2 Cellular::getAccumulatedVector(const CCI_FootBotProximitySensor::TReadings& readings, Real threshold) const {
    CVector2 accumulator;
    for (auto& r : readings) {
        LOG << "Reading " << r.Value;
        if (r.Value >= threshold) {
            LOG << " [add]";
            accumulator += CVector2(r.Value, r.Angle);
        }
        LOG << "\n";
    }
    return accumulator;
}

void Cellular::move(const CDegrees& rotationAngle) {
    auto angleDiff = getControl(rotationAngle);

    if (angleDiff.GetAbsoluteValue() > angleEpsilon) {
        rotateForAnAngle(angleDiff);
    }
    else {
        wheelsEngine->SetLinearVelocity(velocity, velocity);
        lastRotation.SetValue(0);
    }
}

CDegrees Cellular::getControl(const CDegrees& rotationAngle) const {
    Real KP = 0.5;
    Real KD = 0.25;
    auto angleDiff = KP * rotationAngle + KD * (rotationAngle - lastRotation);
    return angleDiff;
}

void Cellular::rotateForAnAngle(const CDegrees& angle) {
    lastControl = angle;
    lastRotation = angle;

    auto rotationDirection = getRotationDirection(angle);
    rotationSpeed = ToRadians(angle).GetAbsoluteValue()
                    * HALF_INTERWHEEL_DISTANCE_IN_CM
                    * TICKS_PER_SEC;
    rotate(rotationDirection);
}

CDegrees Cellular::getOrientationOnXY() {
    CRadians angleX, angleY, angleZ;
    positioningSensor->GetReading().Orientation.ToEulerAngles(angleX, angleY, angleZ);
    CDegrees robotsOrientation = ToDegrees(angleX);
    return robotsOrientation;
}

Cellular::Direction Cellular::getRotationDirection(const CDegrees& obstacleAngle) {
    return obstacleAngle.GetValue() > 0.0f ? Direction::Right : Direction::Left;
}

void Cellular::rotate(Direction rotationDirection) {
    if (rotationDirection == Direction::Right)
        wheelsEngine->SetLinearVelocity(rotationSpeed, -(rotationSpeed * 0.8));
    else if (rotationDirection == Direction::Left)
        wheelsEngine->SetLinearVelocity(-(rotationSpeed * 0.8), rotationSpeed);
}

REGISTER_CONTROLLER(Cellular, "cellular_decomposition_controller")
