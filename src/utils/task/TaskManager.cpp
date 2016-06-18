#include "TaskManager.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <list>
#include <functional>
#include <mutex>
#include <algorithm>

#include "assert.h"

/*
 * HOW TO DISJOINT CELLS?
 *
 * TaskManager:
 *  unassigned robots = robots without a cell
 *  for cell in cells
 *      from the unassigned robots assign closest to cell begining
 *      set the cell explorers - two that are the closest to the explorers begining points
 *      rest assign as a sweepers
 *
 *      explorers
 *          go to the beginning point
 *          if all explorers are near their beginning points
 *              start exploring - follow proper wall
 *          if explorers lose the line of sight
 *              add two new cells
 *              assign them as a sweepers
 *
 *      sweepers
 *          disperse evenly in the one line near edge
 *          move upwards/downwards
 *
 *
 */

using namespace std;
using namespace argos;

mutex handlerAccessMutex;

void TaskManager::addNewCell(CRange<CVector2> limits)
{
    CVector2 leftBottomCorner(limits.GetMax().GetX(), limits.GetMax().GetY());
    CVector2 leftUpperCorner(limits.GetMax().GetX(), limits.GetMin().GetY());

    CVector2 rightBottomCorner(limits.GetMin().GetX(), limits.GetMax().GetY());
    CVector2 rightUpperCorner(limits.GetMin().GetX(), limits.GetMin().GetY());

    availableTasks = {
        {leftBottomCorner,  leftUpperCorner,  Task::Behavior::FollowLeftBoundary,  Task::Status::MoveToBegin},
        {rightBottomCorner, rightUpperCorner, Task::Behavior::FollowRightBoundary, Task::Status::MoveToBegin}
    };
}

void TaskManager::registerHandler(TaskHandler& handler) {
    lock_guard<mutex> guard(handlerAccessMutex);
    handlers.push_back(ref(handler));
}

void TaskManager::unregisterHandler(TaskHandler& handler) {
    lock_guard<mutex> guard(handlerAccessMutex);
    reference_wrapper<TaskHandler> ref(handler);
    auto it = find_if(handlers.begin(), handlers.end(),
                      [&handler](reference_wrapper<TaskHandler>& ref){ return &ref.get() == &handler; });
    if (it != handlers.end())
        handlers.erase(it);
    else
        THROW_ARGOSEXCEPTION("Handler do not exists!");
}

void TaskManager::assignTasks() {
    finishWaitingTasks();
    updateMovingHandlers();
    auto unassignedHandlers = getIdleHandlers();

    LOG << __PRETTY_FUNCTION__ << "\n"
        << "|- " << availableTasks.size() << " available tasks" << "\n"
        << "|- " << handlers.size() << " handlers" << "\n"
        << "|- " << unassignedHandlers.size() << " unassigned handlers" << endl;

    for (auto taskIt = availableTasks.begin(); taskIt != availableTasks.end(); taskIt++) {
        if (unassignedHandlers.size() != 0) {
            unassignedHandlers.front().get().update(*taskIt);
            unassignedHandlers.pop_front();
            taskIt = availableTasks.erase(taskIt);
        }
    }
}

void TaskManager::updateMovingHandlers() const {
    for (auto handler : handlers) {
        auto handlerTask = handler.get().getCurrentTask();
        if (handlerTask.status == Task::Status::MoveToBegin)
            setStatusIfNearGoal(handler.get(), Task::Status::MoveToEnd, handlerTask.begin);
//        if (handlerTask.status == Task::Status::MoveToEnd)
//            setStatusIfNearGoal(handler.get(), Task::Status::Wait, handlerTask.end);
    }
}

list<reference_wrapper<TaskHandler>> TaskManager::getIdleHandlers() const {
    list<reference_wrapper<TaskHandler>> unassignedHandlers;
    for(auto handler : handlers)
        if (handler.get().getCurrentTask().behavior == Task::Behavior::Idle)
            unassignedHandlers.emplace_back(handler);
    return unassignedHandlers;
}

void TaskManager::setStatusIfNearGoal(TaskHandler& handler, const Task::Status& status, const CVector2& goal) const {
    Real minDistance = 0.001f;
    if ((handler.getPostion() - goal).SquareLength() < minDistance) {
        auto task = handler.getCurrentTask();
        task.status = status;
        handler.update(task);
    }
}

void TaskManager::finishWaitingTasks() const {
    Task idleTask = {CVector2(), CVector2(), Task::Behavior::Idle, Task::Status::Wait};
    for(auto handler : handlers) {
        auto handlerTask = handler.get().getCurrentTask();
        if (handlerTask.status == Task::Status::Wait)
            handler.get().update(idleTask);
    }
}