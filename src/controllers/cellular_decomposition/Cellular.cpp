#include "Cellular.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <sstream>

using namespace std;
using namespace argos;

//class DummyLog : public std::stringstream {
//public:
//    void Flush() {}
//};
//static DummyLog dummyLog;
//#define LOG dummyLog

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
    cameraFront = GetSensor<CCI_ColoredBlobPerspectiveCameraSensor>("perspective_camera_front");
    cameraBack = GetSensor<CCI_ColoredBlobPerspectiveCameraSensor>("perspective_camera_back");

//    GetNodeAttributeOrDefault(configuration, "velocity", velocity, velocity);
//    GetNodeAttributeOrDefault(configuration, "min_distance", minDistanceFromObstacle,
//                              minDistanceFromObstacle);

    assert(wheelsEngine != nullptr);
    assert(proximitySensor != nullptr);
    assert(positioningSensor != nullptr);
    assert(lightSensor != nullptr);
    assert(rabRx != nullptr);
    assert(leds != nullptr);
    assert(cameraLeft != nullptr);
    assert(cameraRight != nullptr);
    assert(cameraFront != nullptr);
    assert(cameraBack != nullptr);

    LOG << "Register me : " << this << endl;
    loopFnc.registerToTaskManager(*this);

    factory.reset( new BehaviorFactory(
        { *positioningSensor, *proximitySensor, {*cameraLeft, *cameraRight, *cameraFront, *cameraBack} },
        { *wheelsEngine, *leds }
    ));
    behavior = factory->create(currentTask);
}

void Cellular::Reset() {
    currentTask = Task();
}

void Cellular::Destroy() {
    LOG << "Unregister me : " << this << endl;
    loopFnc.unregisterFromTaskManager(*this);
}

void Cellular::update(Task newTask) {
    TaskHandler::update(newTask);
    LOG << "update task" << endl;
    behavior = factory->create(currentTask);
}

CVector2 Cellular::getPosition() {
    CVector2 position;
    positioningSensor->GetReading().Position.ProjectOntoXY(position);
    return position;
}

bool Cellular::isCriticalPoint() {
    return criticalPointDetected;
}

bool Cellular::isReadyToProceed() {
    return readyToProceed;
}

void Cellular::ControlStep() {
    LOG << "[" << GetId() << "]: " << to_string(currentTask) << "\n";
    switch (currentTask.status) {
        case Task::Status::MoveToBegin:
            behavior->moveToBegin(currentTask.begin);
            break;
        case Task::Status::Prepare:
            behavior->prepare();
            break;
        case Task::Status::Proceed:
            behavior->proceed();
            break;
        case Task::Status::Wait:
            behavior->stop();
    }
    criticalPointDetected = behavior->isCriticalPoint();
    readyToProceed = behavior->isReadyToProceed();
    LOG.Flush();
}


//CColor Cellular::getMyColor() const {
//    if (currentTask.behavior == Task::Behavior::FollowLeftBoundary)
//        return CColor::GREEN;
//    else if (currentTask.behavior == Task::Behavior::FollowRightBoundary)
//        return CColor::RED;
//    THROW_ARGOSEXCEPTION("Robot is not an explorer!");
//}
//
//CColor Cellular::getFellowColor() const {
//    if (currentTask.behavior == Task::Behavior::FollowLeftBoundary)
//        return CColor::RED;
//    else if (currentTask.behavior == Task::Behavior::FollowRightBoundary)
//        return CColor::GREEN;
//    THROW_ARGOSEXCEPTION("Robot is not an explorer!");
//}
//
//void Cellular::explorerMove(const CColor& explorerColor, const CColor& fellowColor,
//                            bool (* isDesiredAngle)(const CDegrees&)) {
//    if (currentTask.status == Task::Status::MoveToBegin)
//        moveToPoint(currentTask.begin);
//    else if (currentTask.status == Task::Status::MoveToEnd) {
//        leds->SetAllColors(explorerColor);
//        bool fellowVisibility = isFellowVisible(fellowColor);
//        auto fellowAngle = getfellowAngle(fellowColor);
//        auto rotationAngle = getRotationAngle();
//        if (rotationAngle.GetAbsoluteValue() > angleEpsilon) {
//            auto angleDiff = getControl(rotationAngle);
//            rotateForAnAngle(angleDiff);
//        } else if (fellowVisibility) {
//            Real frontThreshold = 0.05f;
//            Real frontAngleEpsilon = 2;
//            auto frontProximityVector = getAccumulatedVector(getFrontProximityReadings(), frontThreshold);
//            bool isFellowAtFront = frontProximityVector.SquareLength() > 0;
//            isFellowAtFront &= fellowAngle.GetAbsoluteValue() <= frontAngleEpsilon;
//            if (!isFellowAtFront && isDesiredAngle(fellowAngle))
//                move(rotationAngle);
//            else {
//                LOG << "Fellow at front? " << boolalpha << isFellowAtFront << endl;
//                LOG << "Desired angle? " << boolalpha << isDesiredAngle(fellowAngle) << endl;
//                LOG << "[!] Concave CP!" << endl;
//            }
//        }
//        else
//            LOG << "[!] Convex CP!" << endl;
//    }
//}
//
//bool Cellular::isFellowVisible(const CColor& color) const {
//    auto& leftBlobs = cameraLeft->GetReadings().BlobList;
//    auto& frontBlobs = cameraFront->GetReadings().BlobList;
//    auto& rightBlobs = cameraRight->GetReadings().BlobList;
//
//    auto allCameraBlobs = leftBlobs;
//    allCameraBlobs.insert(allCameraBlobs.end(), frontBlobs.begin(), frontBlobs.end());
//    allCameraBlobs.insert(allCameraBlobs.end(), rightBlobs.begin(), rightBlobs.end());
//
//    for (auto& blob : allCameraBlobs)
//            if (blob->Color == color)
//                return true;
//    return false;
//}
//
//CDegrees Cellular::getfellowAngle(const CColor& color) const {
//    const CDegrees leftCameraOffset(135);
//    const CDegrees frontCameraOffset(45);
//    const CDegrees rightCameraOffset(-45);
//
//    CDegrees fellowAngle;
//    size_t addedAnglesCounter = 0;
//    for (auto& blob : cameraLeft->GetReadings().BlobList)
//            if (blob->Color == color) {
//                LOG << "Left : " << leftCameraOffset - CDegrees((blob->X / 10)) << endl;
//                fellowAngle += leftCameraOffset - CDegrees((blob->X / 10));
//                addedAnglesCounter++;
//            }
//    for (auto& blob : cameraFront->GetReadings().BlobList)
//            if (blob->Color == color) {
//                LOG << "Front : " << frontCameraOffset - CDegrees((blob->X / 10)) << endl;
//                fellowAngle += frontCameraOffset - CDegrees((blob->X / 10));
//                addedAnglesCounter++;
//            }
//    for (auto& blob : cameraRight->GetReadings().BlobList)
//            if (blob->Color == color) {
//                LOG << "Right : " << rightCameraOffset - CDegrees((blob->X / 10)) << endl;
//                fellowAngle += rightCameraOffset - CDegrees((blob->X / 10));
//                addedAnglesCounter++;
//            }
//    fellowAngle /= addedAnglesCounter;
//    return fellowAngle;
//}
//
//void Cellular::stopWheels() const { wheelsEngine->SetLinearVelocity(0, 0); }
//
//void Cellular::moveToPoint(const CVector2& point) {
//    CDegrees rotationAngle = myPositionToPointAngle(point);
//    move(rotationAngle);
//}
//
//CDegrees Cellular::myPositionToPointAngle(const CVector2& point) {
//    CVector2 currentPoint;
//    positioningSensor->GetReading().Position.ProjectOntoXY(currentPoint);
//    CDegrees angle = getAngleBetweenPoints(currentPoint, point);
//    CDegrees robotsOrientation = getOrientationOnXY();
//    auto rotationAngle = (robotsOrientation - angle).SignedNormalize();
//    return rotationAngle;
//}
//
//CDegrees Cellular::getAngleBetweenPoints(const CVector2& a, const CVector2& b) const {
//    auto angle = ToDegrees(ATan2(b.GetY() - a.GetY(), b.GetX() - a.GetX()));
//    return angle;
//}
//
//CDegrees Cellular::getRotationAngle() const {
//    auto leftReadings = getLeftProximityReadings();
//    auto backReadings = getBackProximityReadings();
//    auto rightReadings = getRightProximityReadings();
//    auto frontReadings = getFrontProximityReadings();
//
//    CDegrees sideAngle;
//    CVector2 accumulator;
//    Real frontThreshold = 0.13;
//    Real sideThreshold = 0.1;
//
//    if (currentTask.behavior == Task::Behavior::FollowLeftBoundary) {
//        sideAngle = CDegrees(90);
//        accumulator = getAccumulatedVector(frontReadings, frontThreshold);
//        if (accumulator.SquareLength() == 0) {
//            LOG << "Only left proximity" << "\n";
//            accumulator = getAccumulatedVector(leftReadings, sideThreshold);
//        }
//    }
//    else if (currentTask.behavior == Task::Behavior::FollowRightBoundary) {
//        sideAngle = CDegrees(-90);
//        accumulator = getAccumulatedVector(frontReadings, frontThreshold);
//        if (accumulator.SquareLength() == 0) {
//            LOG << "Only right proximity" << "\n";
//            accumulator = getAccumulatedVector(rightReadings, sideThreshold);
//        }
//    }
//
//    CDegrees rotationAngle = lastRotation;
//    if (accumulator.SquareLength() > 0) {
//        rotationAngle = (sideAngle - ToDegrees(accumulator.Angle())).SignedNormalize();
//    }
//    LOG << "Rotation angle " << rotationAngle << "\n";
//    return rotationAngle;
//}
//
//CCI_FootBotProximitySensor::TReadings Cellular::getFrontProximityReadings() const {
//    const auto& readings = proximitySensor->GetReadings();
//    CCI_FootBotProximitySensor::TReadings frontReadings(readings.begin() + 22, readings.end());
//    frontReadings.insert(frontReadings.end(), readings.begin(), readings.begin() + 2);
//    return frontReadings;
//}
//
//CCI_FootBotProximitySensor::TReadings Cellular::getRightProximityReadings() const {
//    const auto& readings = proximitySensor->GetReadings();
//    CCI_FootBotProximitySensor::TReadings rightReadings(readings.begin() + 16, readings.begin() + 20);
//    return rightReadings;
//}
//
//CCI_FootBotProximitySensor::TReadings Cellular::getBackProximityReadings() const {
//    const auto& readings = proximitySensor->GetReadings();
//    CCI_FootBotProximitySensor::TReadings backReadings(readings.begin() + 10, readings.begin() + 14);
//    return backReadings;
//}
//
//CCI_FootBotProximitySensor::TReadings Cellular::getLeftProximityReadings() const {
//    const auto& readings = proximitySensor->GetReadings();
//    CCI_FootBotProximitySensor::TReadings leftReadings(readings.begin() + 4, readings.begin() + 8);
//    return leftReadings;
//}
//
//CVector2 Cellular::getAccumulatedVector(const CCI_FootBotProximitySensor::TReadings& readings, Real threshold) const {
//    CVector2 accumulator;
//    for (auto& r : readings) {
//        if (r.Value >= threshold) {
//            accumulator += CVector2(r.Value, r.Angle);
//        }
//    }
//    return accumulator;
//}
//
//void Cellular::move(const CDegrees& rotationAngle) {
//    auto angleDiff = getControl(rotationAngle);
//
//    if (angleDiff.GetAbsoluteValue() > angleEpsilon) {
//        rotateForAnAngle(angleDiff);
//    }
//    else {
//        wheelsEngine->SetLinearVelocity(velocity, velocity);
//        lastRotation.SetValue(0);
//    }
//}
//
//CDegrees Cellular::getControl(const CDegrees& rotationAngle) const {
//    Real KP = 0.5;
//    Real KD = 0.25;
//    auto angleDiff = KP * rotationAngle + KD * (rotationAngle - lastRotation);
//    LOG << "Control " << angleDiff << endl;
//    return angleDiff;
//}
//
//void Cellular::rotateForAnAngle(const CDegrees& angle) {
//    lastControl = angle;
//    lastRotation = angle;
//
//    auto rotationDirection = getRotationDirection(angle);
//    rotationSpeed = ToRadians(angle).GetAbsoluteValue()
//                    * HALF_INTERWHEEL_DISTANCE_IN_CM
//                    * TICKS_PER_SEC;
//    rotate(rotationDirection);
//}
//
//CDegrees Cellular::getOrientationOnXY() {
//    CRadians angleX, angleY, angleZ;
//    positioningSensor->GetReading().Orientation.ToEulerAngles(angleX, angleY, angleZ);
//    CDegrees robotsOrientation = ToDegrees(angleX);
//    return robotsOrientation;
//}
//
//Cellular::Direction Cellular::getRotationDirection(const CDegrees& obstacleAngle) {
//    return obstacleAngle.GetValue() > 0.0f ? Direction::Right : Direction::Left;
//}
//
//void Cellular::rotate(Direction rotationDirection) {
//    LOG << "Rotate : " << rotationSpeed << endl;
//    if (rotationDirection == Direction::Right)
//        wheelsEngine->SetLinearVelocity(rotationSpeed, -(rotationSpeed * 0.8));
//    else if (rotationDirection == Direction::Left)
//        wheelsEngine->SetLinearVelocity(-(rotationSpeed * 0.8), rotationSpeed);
//}

REGISTER_CONTROLLER(Cellular, "cellular_decomposition_controller")
