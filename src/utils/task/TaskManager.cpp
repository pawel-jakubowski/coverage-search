#include "TaskManager.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <list>
#include <functional>

#include "assert.h"


using namespace std;
using namespace argos;

void TaskManager::registerHandler(TaskHandler& handler) {
    handlers.push_back(ref(handler));
}

void TaskManager::assignTasks() {
    CVector2 leftBottomCorner(limits.GetMax().GetX(), limits.GetMax().GetY());
    CVector2 leftUpperCorner(limits.GetMax().GetX(), limits.GetMin().GetY());

    CVector2 rightBottomCorner(limits.GetMin().GetX(), limits.GetMax().GetY());
    CVector2 rightUpperCorner(limits.GetMin().GetX(), limits.GetMin().GetY());

    list <Task> availableTasks = {
        {leftBottomCorner,  leftUpperCorner,  Task::Behavior::FollowLeftBoundary,  Task::Status::MoveToBegin},
        {rightBottomCorner, rightUpperCorner, Task::Behavior::FollowRightBoundary, Task::Status::MoveToBegin}
    };

    finishWaitingTasks();
    updateMovingHandlers();
    list<reference_wrapper<TaskHandler>> unassignedHandlers = getIdleHandlers();

    for (auto task : availableTasks) {
        if (unassignedHandlers.size() != 0) {
            unassignedHandlers.front().get().update(task);
            unassignedHandlers.pop_front();
        }
    }
}

void TaskManager::updateMovingHandlers() const {
    for (auto handler : handlers) {
        auto handlerTask = handler.get().getCurrentTask();
        if (handlerTask.status == Task::Status::MoveToBegin)
            setStatusIfNearGoal(handler.get(), Task::Status::MoveToEnd, handlerTask.begin);
        if (handlerTask.status == Task::Status::MoveToEnd)
            setStatusIfNearGoal(handler.get(), Task::Status::Wait, handlerTask.end);
    }
}

list<reference_wrapper<TaskHandler>> TaskManager::getIdleHandlers() const {
    list<reference_wrapper<TaskHandler>> unassignedHandlers;
    for(auto handler : handlers)
        if (handler.get().getCurrentTask().behavior == Task::Behavior::Idle)
            unassignedHandlers.push_back(handler);
    return unassignedHandlers;
}

void TaskManager::setStatusIfNearGoal(TaskHandler& handler, const Task::Status& status, const CVector2& goal) const {
    Real minDistance = 0.01f;
    if ((handler.getPostion() - goal).SquareLength() < minDistance)
        handler.getCurrentTask().status = status;
}

void TaskManager::finishWaitingTasks() const {
    Task idleTask = {CVector2(), CVector2(), Task::Behavior::Idle, Task::Status::Wait};
    for(auto handler : handlers) {
        auto& handlerTask = handler.get().getCurrentTask();
        if (handlerTask.status == Task::Status::Wait)
            handlerTask = idleTask;
    }
}