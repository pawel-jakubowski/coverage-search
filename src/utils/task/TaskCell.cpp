#include "TaskCell.h"
#include <argos3/core/utility/logging/argos_log.h>


using namespace std;
using namespace argos;

static const Real FOOTBOT_BODY_RADIUS = 0.085036758f;
static const Real ARENA_CLEARANCE = FOOTBOT_BODY_RADIUS + 0.05f;
static const Real ROBOT_CLEARANCE_RADIUS = FOOTBOT_BODY_RADIUS + 0.08f;
static const Real ROBOT_CLEARANCE = 2 * ROBOT_CLEARANCE_RADIUS;


TaskCell::TaskCell(argos::CVector2 beginning)
    : beginning(beginning)
    , end(beginning)
    , limits(beginning, end)
    , explorers{nullptr, nullptr}
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
        LOG << "Cell is finished!" << endl;
        return;
    }

    if (!isReady()) {
        LOG << "Cell is not ready!" << endl;
        return;
    }

    if (!started && !areExplorersAtBeginning())
        moveExplorersToBeginning();
    else if (!prepared && !areExplorersReadyToProceed())
        prepareExplorers();
    else
        proceedExplorers();

    LOG << "=====================================" << endl;
}

bool TaskCell::areExplorersReadyToProceed() const {
    return explorers.at(0)->isReadyToProceed() && explorers.at(1)->isReadyToProceed();
}

void TaskCell::finishCell() {
    LOG << "Explorer report CP!" << endl;
    Task idleTask{CVector2(), CVector2(), Task::Behavior::Idle, Task::Status::Wait};
    explorers.at(0)->update(idleTask);
    explorers.at(1)->update(idleTask);
    explorers = {nullptr, nullptr};
    finished = true;
}

void TaskCell::updateCellLimits() {
    auto x = (
                 explorers.at(0)->getPosition().GetX() +
                 explorers.at(1)->getPosition().GetX()
             ) / 2;
    auto y = (
                 explorers.at(0)->getPosition().GetY() +
                 explorers.at(1)->getPosition().GetY()
             ) / 2;
    end = CVector2(x,y);

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
    return isNear(*explorers.at(index), explorers.at(index)->getCurrentTask().begin);
}

void TaskCell::addLeftExplorer(TaskHandler& e) {
    if (e.getCurrentTask().behavior != Task::Behavior::FollowLeftBoundary)
        THROW_ARGOSEXCEPTION("Left explorer do not have FLEFT task!");
    explorers[Explorer::Left] = &e;
}

void TaskCell::addRightExplorer(TaskHandler& e) {
    if (e.getCurrentTask().behavior != Task::Behavior::FollowRightBoundary)
        THROW_ARGOSEXCEPTION("Right explorer do not have FRIGHT task!");
    explorers[Explorer::Right] = &e;
}

bool TaskCell::isReady() const {
    return explorers[Left] != nullptr && explorers[Right] != nullptr;
}

bool TaskCell::isFinished() const {
    return finished;
}

bool TaskCell::isNear(TaskHandler& handler, const CVector2& point) const {
    Real minDistance = 0.001f;
    return (handler.getPosition() - point).SquareLength() < minDistance;
}

void TaskCell::moveExplorersToBeginning() {
    LOG << "Move explorers to begin!" << endl;
    updateExplorers(Task::Status::MoveToBegin);
    Explorer index = Left;
    if (isExplorerNearBeginning(static_cast<Explorer>(index)))
        updateExplorerStatus(static_cast<Explorer>(index), Task::Status::Wait);
    index = Right;
    if (isExplorerNearBeginning(static_cast<Explorer>(index)))
        updateExplorerStatus(static_cast<Explorer>(index), Task::Status::Wait);
}

void TaskCell::prepareExplorers() {
    LOG << "Prepare explorers!" << endl;
    updateExplorers(Task::Status::Prepare);
    started = true;
}

void TaskCell::proceedExplorers() {
    LOG << "Proceed explorers!" << endl;
    updateExplorers(Task::Status::Proceed);
    prepared = true;

    if (explorers.at(Left)->isCriticalPoint() && explorers.at(Right)->isCriticalPoint()) {
        updateExplorers(Task::Status::Wait);
        finishCell();
        return;
    }

    if (explorers.at(Left)->getPosition().GetY() < explorers.at(Right)->getPosition().GetY())
        updateExplorerStatus(Left, Task::Status::Wait);
    else if (explorers.at(Right)->getPosition().GetY() < explorers.at(Left)->getPosition().GetY())
        updateExplorerStatus(Right, Task::Status::Wait);

    updateCellLimits();
}
