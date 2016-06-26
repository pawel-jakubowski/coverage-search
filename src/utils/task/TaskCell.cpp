#include "TaskCell.h"
#include <argos3/core/utility/logging/argos_log.h>


void checkIfReady();

using namespace std;
using namespace argos;

TaskCell::TaskCell(array<CVector2, 2> corners) : explorers{nullptr, nullptr}  {
    if (corners.at(0) < corners.at(1))
        limits = CRange<CVector2>(corners.at(0), corners.at(1));
    else
        limits = CRange<CVector2>(corners.at(1), corners.at(0));
}

void TaskCell::update() {
    if (!isReady()) {
        LOG << "Cell is not ready!" << endl;
        return;
    }

    if (finished) {
        LOG << "Cell is finished!" << endl;
        return;
    }

    if (!areExplorersAtBeginning()) {
        LOG << "Move explorers to begin!" << endl;
        moveExplorersToBegin();
    }
    else {
        LOG << "Proceed explorers!" << endl;
        proceedExplorers();
    }

    unsigned index = 0;
    for (auto explorer : explorers) {
        if (explorer->getCurrentTask().status == Task::Status::MoveToBegin &&
            isExplorerNearBeginning(static_cast<Explorer>(index)))
            stopExplorer(static_cast<Explorer>(index));
        else if (explorer->getCurrentTask().status == Task::Status::Proceed) {
            if (explorer->isCriticalPoint()) {
                LOG << "Explorer report CP!" << endl;
                finished = true;
                return;
            }
            updateCellLimits(explorer);
        }
        index++;
    }

    LOG << "Cell : " << limits << "\n"
        << "=====================================" << endl;
}

void TaskCell::stopExplorer(Explorer index) {
    auto task = explorers.at(index)->getCurrentTask();
    task.status = Task::Status::Wait;
    explorers.at(index)->update(task);
}

void TaskCell::updateCellLimits(TaskHandler* explorer) {
    auto position = explorer->getPosition();
    if (!limits.WithinMinBoundIncludedMaxBoundIncluded(position)) {
            auto limitsMin = limits.GetMin();
            auto limitsMax = limits.GetMax();

            if (position.GetX() < limitsMin.GetX())
                limitsMin.SetX(position.GetX());
            else if (position.GetX() > limitsMax.GetX())
                limitsMax.SetX(position.GetX());

            if (position.GetY() < limitsMin.GetY())
                limitsMin.SetY(position.GetY());
            else if (position.GetY() > limitsMax.GetY())
                limitsMax.SetY(position.GetY());

            limits.SetMin(limitsMin);
            limits.SetMax(limitsMax);
        }
}

void TaskCell::moveExplorersToBegin() {
    moveExplorerToBegin(Left);
    moveExplorerToBegin(Right);
}

void TaskCell::moveExplorerToBegin(Explorer index) {
    auto task = explorers.at(index)->getCurrentTask();
    task.status = Task::Status::MoveToBegin;
    explorers.at(index)->update(task);
}

void TaskCell::proceedExplorers() {
    proceedExplorer(Left);
    proceedExplorer(Right);
}

void TaskCell::proceedExplorer(Explorer index) {
    auto task = explorers.at(index)->getCurrentTask();
    task.status = Task::Status::Proceed;
    explorers.at(index)->update(task);
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
