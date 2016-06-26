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


//class DummyLog : public std::stringstream {
//public:
//    void Flush() {}
//};
//static DummyLog dummyLog;
//#define LOG dummyLog



static const Real FOOTBOT_BODY_RADIUS = 0.085036758f;
static const Real ARENA_CLEARANCE = FOOTBOT_BODY_RADIUS + 0.05f;
static const Real ROBOT_CLEARANCE_RADIUS = 0.4f;

mutex handlerAccessMutex;

void TaskManager::addNewCell(CRange<CVector2> limits)
{
    CVector2 leftBottomCorner(limits.GetMax().GetX() - ARENA_CLEARANCE, limits.GetMax().GetY() - ARENA_CLEARANCE);
    CVector2 rightBottomCorner(limits.GetMin().GetX() + ARENA_CLEARANCE, limits.GetMax().GetY() - ARENA_CLEARANCE);

    CVector2 sweepRightBottomCorner(limits.GetMin().GetX() + ARENA_CLEARANCE,
                                    limits.GetMax().GetY() - ARENA_CLEARANCE + ROBOT_CLEARANCE_RADIUS);

    cells.emplace_back(array<CVector2,2>{leftBottomCorner, rightBottomCorner});

    availableTasks = {
        {leftBottomCorner, leftBottomCorner, Task::Behavior::FollowLeftBoundary,  Task::Status::MoveToBegin},
        {rightBottomCorner, rightBottomCorner, Task::Behavior::FollowRightBoundary, Task::Status::MoveToBegin}
//        {leftBottomCorner, sweepRightBottomCorner, Task::Behavior::Sweep,  Task::Status::MoveToBegin},
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
        LOGERR << "[WARNING] Handler do not exists! " << &handler << endl;
}

void TaskManager::assignTasks() {
    if (!ready)
        initialize();

//    finishWaitingTasks();
    updateMovingHandlers();
    auto unassignedHandlers = getIdleHandlers();

    LOG << __PRETTY_FUNCTION__ << "\n"
        << "|- " << availableTasks.size() << " available tasks" << "\n"
        << "|- " << handlers.size() << " handlers" << "\n"
        << "|- " << unassignedHandlers.size() << " unassigned handlers" << endl;

    for (auto taskIt = availableTasks.begin(); taskIt != availableTasks.end(); taskIt++) {
        if (unassignedHandlers.size() != 0) {
            auto closestHandler = unassignedHandlers.begin();
            auto closestHandlerDistance = (closestHandler->get().getPosition() - taskIt->begin).SquareLength();
            for(auto handlerIt = unassignedHandlers.begin(); handlerIt != unassignedHandlers.end(); handlerIt++) {
                auto distance = (handlerIt->get().getPosition() - taskIt->begin).SquareLength();
                if (distance < closestHandlerDistance) {
                    closestHandler = handlerIt;
                    closestHandlerDistance = distance;
                }
            }

            closestHandler->get().update(*taskIt);
            unassignedHandlers.erase(closestHandler);
            taskIt = availableTasks.erase(taskIt);

            if (closestHandler->get().getCurrentTask().behavior == Task::Behavior::FollowLeftBoundary)
                cells.at(0).addLeftExplorer(*closestHandler);
            else if (closestHandler->get().getCurrentTask().behavior == Task::Behavior::FollowRightBoundary)
                cells.at(0).addRightExplorer(*closestHandler);
        }
    }

    for (auto& cell : cells)
        cell.update();
}

void TaskManager::initialize() {
    if (cells.size() != 1)
        THROW_ARGOSEXCEPTION("During initialization only one cell should be available!");

    auto unassignedHandlers = getIdleHandlers();
    CVector2 start = cells.at(0).getLimits().GetMax();
    Real minX = cells.at(0).getLimits().GetMin().GetX();

    Task task = {CVector2(start.GetX() - ARENA_CLEARANCE, start.GetY()), start, Task::Behavior::Sweep,
                          Task::Status::MoveToBegin};
    for (auto handler : unassignedHandlers) {
        task.begin.SetX( task.begin.GetX() - ROBOT_CLEARANCE_RADIUS );
        if (task.begin.GetX() < minX) {
            task.begin.SetX(start.GetX());
            task.begin.SetY( task.begin.GetY() - ROBOT_CLEARANCE_RADIUS );
        }
        handler.get().update(task);
    }

    if (getIdleHandlers().size() > 0)
        THROW_ARGOSEXCEPTION("All robots should have assigned start point!");

    for (auto handler : handlers)
        if (isNearGoal(handler.get(), handler.get().getCurrentTask().begin)) {
            auto handlerTask = handler.get().getCurrentTask();
            handlerTask.status = Task::Status::Wait;
            handler.get().update(handlerTask);
            ready &= true;
        }
}

void TaskManager::updateMovingHandlers() {
    for (auto handler : handlers) {
        auto handlerTask = handler.get().getCurrentTask();
        if (handlerTask.behavior == Task::Behavior::Sweep)
            updateSweeperTask(handler);
        else if (handlerTask.behavior == Task::Behavior::FollowLeftBoundary ||
            handlerTask.behavior == Task::Behavior::FollowRightBoundary)
            updateExplorerTask(handler);
    }
}

void TaskManager::updateSweeperTask(TaskHandler& handler) {
    auto handlerTask = handler.getCurrentTask();
    if (handlerTask.status == Task::Status::MoveToBegin) {
        if (isNearGoal(handler, handlerTask.begin)) {
            if (handlerTask.begin.GetY() == handlerTask.end.GetY())
                handlerTask.begin.SetY(handlerTask.begin.GetY() - ROBOT_CLEARANCE_RADIUS);
            else {
                handlerTask.end.SetY(handlerTask.end.GetY() - ROBOT_CLEARANCE_RADIUS);
                handlerTask.status = Task::Status::Proceed;
            }
            handler.update(handlerTask);
        }
    }
    else if (handlerTask.status == Task::Status::Proceed) {
        if (isNearGoal(handler, handlerTask.end)) {
            if (handlerTask.end.GetY() == handlerTask.begin.GetY())
                handlerTask.end.SetY(handlerTask.end.GetY() - ROBOT_CLEARANCE_RADIUS);
            else {
                handlerTask.begin.SetY(handlerTask.begin.GetY() - ROBOT_CLEARANCE_RADIUS);
                handlerTask.status = Task::Status::MoveToBegin;
            }
            handler.update(handlerTask);
        }
    }
}

void TaskManager::updateExplorerTask(TaskHandler& handler) {
    auto handlerTask = handler.getCurrentTask();
    if (handlerTask.status == Task::Status::MoveToBegin)
        setStatusIfNearGoal(handler, Task::Status::Proceed, handlerTask.begin);
//        if (handlerTask.status == Task::Status::Proceed)
//            setStatusIfNearGoal(handler.get(), Task::Status::Wait, handlerTask.end);
}

list<reference_wrapper<TaskHandler>> TaskManager::getIdleHandlers() const {
    list<reference_wrapper<TaskHandler>> unassignedHandlers;
    for(auto handler : handlers)
        if (handler.get().getCurrentTask().behavior == Task::Behavior::Idle)
            unassignedHandlers.emplace_back(handler);
    return unassignedHandlers;
}

void TaskManager::setStatusIfNearGoal(TaskHandler& handler, const Task::Status& status, const CVector2& goal) {
    if (isNearGoal(handler, goal)) {
        auto task = handler.getCurrentTask();
        task.status = status;
        handler.update(task);
    }
}

bool TaskManager::isNearGoal(TaskHandler& handler, const CVector2& goal) const {
    Real minDistance = 0.001f;
    return (handler.getPosition() - goal).SquareLength() < minDistance;
}

void TaskManager::finishWaitingTasks() {
    Task idleTask = {CVector2(), CVector2(), Task::Behavior::Idle, Task::Status::Wait};
    for(auto handler : handlers) {
        auto handlerTask = handler.get().getCurrentTask();
        if (handlerTask.status == Task::Status::Wait)
            handler.get().update(idleTask);
    }
}