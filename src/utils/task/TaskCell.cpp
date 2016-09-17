#include "TaskCell.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <assert.h>


using namespace std;
using namespace argos;

static const Real FOOTBOT_BODY_RADIUS = 0.085036758f;
static const Real ARENA_CLEARANCE = FOOTBOT_BODY_RADIUS + 0.05f;
static const Real ROBOT_CLEARANCE_RADIUS = FOOTBOT_BODY_RADIUS + 0.06f;
static const Real ROBOT_CLEARANCE = 2 * ROBOT_CLEARANCE_RADIUS;


TaskCell::TaskCell(argos::CVector2 beginning)
    : beginning(beginning)
    , end(beginning)
    , limits(beginning, end)
    , explorers{nullptr, nullptr}
    , explorersDistanceThreshold(0.15)
{}

list<Task> TaskCell::getExplorersTasks() const {
    CVector2 left(beginning.GetX() + ROBOT_CLEARANCE_RADIUS, beginning.GetY());
    CVector2 right(beginning.GetX() - ROBOT_CLEARANCE_RADIUS, beginning.GetY());
    return {
        {left, left, Task::Behavior::FollowLeftBoundary,  Task::Status::MoveToBegin},
        {right, right, Task::Behavior::FollowRightBoundary,  Task::Status::MoveToBegin}
    };
}

void TaskCell::update() {
    if (finished) {
//        LOG << (void*)(this) << " Cell is finished!" << endl;
        freeExplorers();
        return;
    }

    if (!isReady()) {
//        LOG << (void*)(this) << " Cell is not ready!" << endl;
        return;
    }

    if (!started && !areExplorersAtBeginning())
        moveExplorersToBeginning();
    else if (!prepared && !areExplorersReadyToProceed())
        prepareExplorers();
    else
        proceedExplorers();
}

void TaskCell::freeExplorer(Explorer index) {
    if (explorers.at(index) == nullptr)
        return;

    CVector2 reversePoint(explorers.at(index)->getPosition().GetX(),
                          explorers.at(index)->getCurrentTask().begin.GetY());
    if (!forwardConvexCP || isNear(*explorers.at(index), reversePoint)) {
        explorers.at(index)->update(Task());
        explorers.at(index) = nullptr;
    }
}

void TaskCell::finish(CVector2 end) {
    if (!finished) {
        finishCell();

        if (forwardConvexCP)
            setRevertTaskForExplorers();
        else
            freeExplorers();
    }
    this->end = end;
}

void TaskCell::freeExplorers() {
    freeExplorer(Left);
    freeExplorer(Right);
}

bool TaskCell::areExplorersReadyToProceed() const {
    return explorers.at(Left)->isReadyToProceed() && explorers.at(Right)->isReadyToProceed();
}

void TaskCell::finishCell() {
//    LOG << "Explorer report CP!" << endl;
    auto explorersDistance = (explorers.at(Left)->getPosition() - explorers.at(Right)->getPosition()).SquareLength();
    auto explorersDistanceOnY = explorers.at(Left)->getPosition().GetY() - explorers.at(Right)->getPosition().GetY();
//    LOG << "Explorers dist: " << explorersDistance << ", "
//        << "Explorers dist on Y: " << explorersDistanceOnY << ", "
//        << "Explorers forward cp: [" << boolalpha << explorers.at(Left)->isForwardConvexCP() << ", "
//        << explorers.at(Right)->isForwardConvexCP() << "]\n";
    forwardConvexCP = explorers.at(Left)->isForwardConvexCP() ||
        explorers.at(Right)->isForwardConvexCP() ||
        fabs(explorersDistanceOnY) > explorersDistanceThreshold;
    reverseConvexCP = !forwardConvexCP && explorersDistance >= ROBOT_CLEARANCE;
    concaveCP = explorers.at(Left)->isConcaveCP() || explorers.at(Right)->isConcaveCP();
    finished = true;
}

void TaskCell::setRevertTaskForExplorers() {
    setRevertTaskForExplorer(Left);
    setRevertTaskForExplorer(Right);
}

void TaskCell::setRevertTaskForExplorer(Explorer index) {
    Task revertTask = explorers.at(index)->getCurrentTask();
    revertTask.status = Task::Status::MoveToBegin;
    revertTask.begin = explorers.at(index)->getPosition();
    auto newY = end.GetY() + ROBOT_CLEARANCE;
    if (newY > limits.GetMax().GetY())
        newY = limits.GetMax().GetY();
    revertTask.begin.SetY(newY);
    explorers.at(index)->update(revertTask);
}

void TaskCell::updateCellLimits() {
    end = (explorers.at(Left)->getPosition() + explorers.at(Right)->getPosition()) / 2;
    for (auto e : explorers) {
        auto x = e->getPosition().GetX();
        auto y = e->getPosition().GetY();
        if (x < limits.GetMin().GetX())
            limits.SetMin(CVector2(x, limits.GetMin().GetY()));
        else if (x > limits.GetMax().GetX())
            limits.SetMax(CVector2(x, limits.GetMax().GetY()));

        if (y < limits.GetMin().GetY())
            limits.SetMin(CVector2(limits.GetMin().GetX(), y));
        else if (y > limits.GetMax().GetY())
            limits.SetMax(CVector2(limits.GetMax().GetX(), y));
    }
}

void TaskCell::updateExplorers(Task::Status status) {
    updateExplorerStatus(Left, status);
    updateExplorerStatus(Right, status);
}

void TaskCell::updateExplorerStatus(Explorer index, Task::Status status) {
    auto task = explorers.at(index)->getCurrentTask();
    if (task.status != status) {
        task.status = status;
        explorers.at(index)->update(task);
    }
}

bool TaskCell::areExplorersAtBeginning() const {
    return isExplorerNearBeginning(Left) && isExplorerNearBeginning(Right);
}

bool TaskCell::isExplorerNearBeginning(Explorer index) const {
    assert(explorers.at(index) != nullptr);
    return isNear(*explorers.at(index), explorers.at(index)->getCurrentTask().begin);
}

void TaskCell::addLeftExplorer(TaskHandler& e) {
//    LOG << (void*)(this) << " add left explorer!\n";
//    if (explorers.at(Left) != nullptr)
//        THROW_ARGOSEXCEPTION("Left explorer is already assigned!");
    if (e.getCurrentTask().behavior != Task::Behavior::FollowLeftBoundary)
        THROW_ARGOSEXCEPTION("Left explorer do not have FLEFT task!");
    explorers[Explorer::Left] = &e;
}

void TaskCell::addRightExplorer(TaskHandler& e) {
//    LOG << (void*)(this) << " add right explorer!\n";
//    if (explorers.at(Right) != nullptr)
//        THROW_ARGOSEXCEPTION("Right explorer is already assigned!");
    if (e.getCurrentTask().behavior != Task::Behavior::FollowRightBoundary)
        THROW_ARGOSEXCEPTION("Right explorer do not have FRIGHT task!");
    explorers[Explorer::Right] = &e;
}

bool TaskCell::isReady() const {
    return explorers[Left] != nullptr && explorers[Right] != nullptr;
}

bool TaskCell::isFinished() const {
    return finished && explorers.at(Left) == nullptr && explorers.at(Right) == nullptr;
}

bool TaskCell::isReverseConvex() const {
    return reverseConvexCP;
}

bool TaskCell::isForwardConvex() const {
    return forwardConvexCP;
}

bool TaskCell::isConcave() const {
    return concaveCP;
}

bool TaskCell::isNear(TaskHandler& handler, const CVector2& point) const {
    Real minDistance = 0.001f;
    return (handler.getPosition() - point).SquareLength() < minDistance;
}

void TaskCell::moveExplorersToBeginning() {
//    LOG << "Move explorers to begin!";
    updateExplorers(Task::Status::MoveToBegin);
    Explorer index = Left;
    if (isExplorerNearBeginning(static_cast<Explorer>(index)))
        updateExplorerStatus(static_cast<Explorer>(index), Task::Status::Wait);
    index = Right;
    if (isExplorerNearBeginning(static_cast<Explorer>(index)))
        updateExplorerStatus(static_cast<Explorer>(index), Task::Status::Wait);
//    LOG << endl;
}

void TaskCell::prepareExplorers() {
//    LOG << "Prepare explorers!";
    updateExplorers(Task::Status::Prepare);
    updateCellLimits();
    started = true;
//    LOG << endl;
}

void TaskCell::proceedExplorers() {
//    LOG << "Proceed explorers!";
    updateExplorers(Task::Status::Proceed);

    auto explorersDistanceOnY = explorers.at(Left)->getPosition().GetY() - explorers.at(Right)->getPosition().GetY();

//    LOG << (void*)(this) << ": "
//        << "L[" << explorers.at(Left)->isCriticalPoint() << ", " << explorers.at(Left)->isForwardConvexCP() << "], "
//        << "R[" << explorers.at(Right)->isCriticalPoint() << ", " << explorers.at(Right)->isForwardConvexCP() << "], "
//        << explorersDistanceOnY << "\n";
    if (!prepared)
        prepared = true;
    else if (
            (explorers.at(Left)->isForwardConvexCP() || explorers.at(Right)->isForwardConvexCP()) ||
            (explorers.at(Left)->isCriticalPoint() && explorers.at(Right)->isCriticalPoint()) ||
            (fabs(explorersDistanceOnY) > explorersDistanceThreshold)
        ) {
        updateExplorers(Task::Status::Wait);
        finishCell();
        if (forwardConvexCP)
            setRevertTaskForExplorers();
        else
            freeExplorers();
        return;
    }

    auto leftRightDistOnY = explorers.at(Left)->getPosition().GetY() - explorers.at(Right)->getPosition().GetY();
    auto distEpsilon = 0.01f;
    if (leftRightDistOnY < -distEpsilon)
        updateExplorerStatus(Left, Task::Status::Wait);
    else if (leftRightDistOnY > distEpsilon)
        updateExplorerStatus(Right, Task::Status::Wait);

    updateCellLimits();
//    LOG << endl;
}
