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

    Sensors s = { *positioningSensor, *proximitySensor, {*cameraLeft, *cameraRight, *cameraFront, *cameraBack} };
    Actuators a = { *wheelsEngine, *leds };
    factory.reset( new BehaviorFactory(s,a));
    behavior = factory->create(currentTask);
    avoidBehavior = make_shared<AvoidObstacleBehavior>(s,a);
}

void Cellular::Reset() {
    currentTask = Task();
}

void Cellular::Destroy() {
    LOG << "Unregister me : " << this << endl;
    loopFnc.unregisterFromTaskManager(*this);
}

void Cellular::update(Task newTask) {
    LOG << "[update task]";
    if (currentTask.behavior != newTask.behavior)
        behavior = factory->create(newTask);
    TaskHandler::update(newTask);
}

CVector2 Cellular::getPosition() const {
    CVector2 position;
    positioningSensor->GetReading().Position.ProjectOntoXY(position);
    return position;
}

bool Cellular::isCriticalPoint() const {
    return criticalPointDetected;
}

bool Cellular::isForwardConvexCP() const {
    return criticalPointDetected && forwardConvexCPDetected;
}

bool Cellular::isReadyToProceed() const {
    return readyToProceed;
}

void Cellular::ControlStep() {
    LOG << "[" << GetId() << "]: " << to_string(currentTask) << "\n";
    if (currentTask.status == Task::Status::Wait)
        behavior->stop();
    else {
        auto velocity = runBehavior();
        if (!avoidBehavior->isRoadClear(velocity))
            velocity = avoidBehavior->proceed();
    }

    criticalPointDetected = behavior->isCriticalPoint();
    forwardConvexCPDetected = behavior->isForwardConvexCP();
    readyToProceed = behavior->isReadyToProceed();

    LOG.Flush();
}

CVector2 Cellular::runBehavior() {
    switch (currentTask.status) {
        case Task::Status::MoveToBegin:
            return behavior->moveToBegin(currentTask.begin);
        case Task::Status::Prepare:
            return behavior->prepare();
        case Task::Status::Proceed:
            return behavior->proceed();
    }
    return CVector2();
}

REGISTER_CONTROLLER(Cellular, "cellular_decomposition_controller")
