#include "TaskCell.h"
#include <argos3/core/utility/logging/argos_log.h>


using namespace std;
using namespace argos;

TaskCell::TaskCell(argos::CVector2 beginning) : beginning(beginning), end(beginning), explorers{nullptr, nullptr}
{}

void TaskCell::update() {
    if (finished) {
        LOG << "Cell is finished!" << endl;
        return;
    }

    if (!isReady()) {
        LOG << "Cell is not ready!" << endl;
        return;
    }

    if (!started && !areExplorersAtBeginning()) {
        LOG << "Move explorers to begin!" << endl;
        updateExplorers(Task::Status::MoveToBegin);
    }
    else if (!prepared && !areExplorersReadyToProceed()) {
        LOG << "Prepare explorers!" << endl;
        updateExplorers(Task::Status::Prepare);
        started = true;
    }
    else {
        LOG << "Proceed explorers!" << endl;
        updateExplorers(Task::Status::Proceed);
        prepared = true;
    }

    unsigned index = 0;
    for (auto explorer : explorers) {
        if (explorer->getCurrentTask().status == Task::Status::MoveToBegin &&
            isExplorerNearBeginning(static_cast<Explorer>(index)))
            updateExplorerStatus(static_cast<Explorer>(index), Task::Status::Wait);
        else if (explorer->getCurrentTask().status == Task::Status::Proceed) {
            if (explorer->isCriticalPoint()) {
                finishCell();
                return;
            }
            updateCellLimits(explorer);
        }
        index++;
    }

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

void TaskCell::updateCellLimits(TaskHandler* explorer) {
    auto x = (
                 explorers.at(0)->getPosition().GetX() +
                 explorers.at(1)->getPosition().GetX()
             ) / 2;
    auto y = (
                 explorers.at(0)->getPosition().GetY() +
                 explorers.at(1)->getPosition().GetY()
             ) / 2;
    end = CVector2(x,y);
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

bool TaskCell::isNear(TaskHandler& handler, const CVector2& point) const {
    Real minDistance = 0.001f;
    return (handler.getPosition() - point).SquareLength() < minDistance;
}
