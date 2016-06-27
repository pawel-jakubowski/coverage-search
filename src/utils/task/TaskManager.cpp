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
static const Real ROBOT_CLEARANCE_RADIUS = FOOTBOT_BODY_RADIUS + 0.08f;
static const Real ROBOT_CLEARANCE = 2 * ROBOT_CLEARANCE_RADIUS;

mutex handlerAccessMutex;

void TaskManager::init(CRange<CVector2> limits) {
    this->limits = CRange<CVector2>(
        CVector2(limits.GetMin().GetX() + ARENA_CLEARANCE, limits.GetMin().GetY() + ARENA_CLEARANCE),
        CVector2(limits.GetMax().GetX() - ARENA_CLEARANCE, limits.GetMax().GetY() - ARENA_CLEARANCE)
    );
}

void TaskManager::addNewCell(CVector2 beginning)
{
    CVector2 leftBottomCorner(ROBOT_CLEARANCE_RADIUS, beginning.GetY());
    CVector2 rightBottomCorner(-ROBOT_CLEARANCE_RADIUS, beginning.GetY());

//    CVector2 sweepRightBottomCorner(limits.GetMin().GetX() + ARENA_CLEARANCE,
//                                    limits.GetMax().GetY() - ARENA_CLEARANCE + ROBOT_CLEARANCE_RADIUS);

    cells.emplace_back(CVector2(0, beginning.GetY()));

    availableTasks.push_back(
        {leftBottomCorner, leftBottomCorner, Task::Behavior::FollowLeftBoundary,  Task::Status::MoveToBegin});
    availableTasks.push_back(
        {rightBottomCorner, rightBottomCorner, Task::Behavior::FollowRightBoundary, Task::Status::MoveToBegin});
//        {leftBottomCorner, sweepRightBottomCorner, Task::Behavior::Sweep,  Task::Status::MoveToBegin}
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
            auto closestHandler = getClosestHandler(unassignedHandlers, *taskIt);
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

TaskManager::HandlersList::const_iterator TaskManager::getClosestHandler(const HandlersList& handlers, const Task&
task)
const {
    auto closestHandler = handlers.begin();
    auto closestHandlerDistance = (closestHandler->get().getPosition() - task.begin).SquareLength();
    for(auto handlerIt = handlers.begin(); handlerIt != handlers.end(); handlerIt++) {
        auto distance = (handlerIt->get().getPosition() - task.begin).SquareLength();
        if (distance < closestHandlerDistance) {
            closestHandler = handlerIt;
            closestHandlerDistance = distance;
        }
    }
    return closestHandler;
}

void TaskManager::initialize() {
    if (cells.size() != 0)
        THROW_ARGOSEXCEPTION("During initialization none cell should be available!");

    auto unassignedHandlers = getIdleHandlers();
    CVector2 start = limits.GetMax();
    Real minX = limits.GetMin().GetX();

    LOG << __PRETTY_FUNCTION__ << ": start " << start << endl;
    Task task = {
        CVector2(start.GetX() + ROBOT_CLEARANCE, start.GetY()), start,
        Task::Behavior::Sweep, Task::Status::MoveToBegin
    };
    while (unassignedHandlers.size() > 0) {
        task.begin.SetX( task.begin.GetX() - ROBOT_CLEARANCE );
        if (task.begin.GetX() < minX) {
            task.begin.SetX(start.GetX());
            task.begin.SetY( task.begin.GetY() - ROBOT_CLEARANCE );
            initialLineWidth += ROBOT_CLEARANCE;
        }
        LOG << __PRETTY_FUNCTION__ << ": Assign " << to_string(task) << endl;
        auto handler = getClosestHandler(unassignedHandlers, task);
        handler->get().update(task);
        unassignedHandlers.erase(handler);
    }

    if (getIdleHandlers().size() > 0)
        THROW_ARGOSEXCEPTION("All robots should have assigned start point!");

    unsigned handlersAtStart = 0;
    for (auto handler : handlers)
        if (isNearGoal(handler.get(), handler.get().getCurrentTask().begin)) {
            auto handlerTask = handler.get().getCurrentTask();
            handlerTask.status = Task::Status::Wait;
            handler.get().update(handlerTask);
            handlersAtStart++;
        }
    LOG << __PRETTY_FUNCTION__ << ": " << handlersAtStart << " handlers initizalized!" << endl;

    if (handlers.size() == handlersAtStart) {
        finishWaitingTasks();
        addNewCell(CVector2(limits.GetMax().GetX(), limits.GetMax().GetY() - initialLineWidth - ROBOT_CLEARANCE));
        ready = true;
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

TaskManager::HandlersList TaskManager::getIdleHandlers() const {
    HandlersList unassignedHandlers;
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