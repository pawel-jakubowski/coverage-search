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
static const Real ROBOT_CLEARANCE_RADIUS = FOOTBOT_BODY_RADIUS + 0.06f;
static const Real ROBOT_CLEARANCE = 2 * ROBOT_CLEARANCE_RADIUS;

mutex handlerAccessMutex;
mutex addNewCellMutex;

void TaskManager::init(CRange<CVector2> limits) {
    this->limits = CRange<CVector2>(
        CVector2(limits.GetMin().GetX() + ARENA_CLEARANCE, limits.GetMin().GetY() + ARENA_CLEARANCE),
        CVector2(limits.GetMax().GetX() - ARENA_CLEARANCE, limits.GetMax().GetY() - ARENA_CLEARANCE)
    );

    ready = false;
    initialLineWidth = 0;
    graph = ReebGraph();
    availableTasks.empty();
}

void TaskManager::addNewCell(CVector2 beginning, int startNode)
{
    auto cellId = graph.addEdge(beginning, startNode);
    auto explorersTasks = graph.getCell(cellId).getExplorersTasks();
    for (auto task : explorersTasks)
        availableTasks.push({task, cellId});
    LOG << "Cell " << cellId << " added! (" << beginning << ")" << endl;
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

    updateCells();

//    finishWaitingTasks();
    updateMovingHandlers();
    auto unassignedHandlers = getIdleWaitingHandlers();

    LOG << "TaskManager: ["
        << availableTasks.size() << " available tasks], ["
        << handlers.size() << " handlers], ["
        << unassignedHandlers.size() << " unassigned handlers]" << endl;

    while(availableTasks.size() > 0 && unassignedHandlers.size() > 0) {
        auto& task = availableTasks.front().first;
        auto cellId = availableTasks.front().second;

        if (!graph.getCell(cellId).isFinished()) {
            LOG << "Task " << to_string(task);
            auto closestHandler = getClosestHandler(unassignedHandlers, task);
            closestHandler->get().update(task);
            unassignedHandlers.erase(closestHandler);

            LOG << " assigned to cell " << cellId << endl;
            if (closestHandler->get().getCurrentTask().behavior == Task::Behavior::FollowLeftBoundary)
                graph.getCell(cellId).addLeftExplorer(*closestHandler);
            else if (closestHandler->get().getCurrentTask().behavior == Task::Behavior::FollowRightBoundary)
                graph.getCell(cellId).addRightExplorer(*closestHandler);
        }

        availableTasks.pop();
    }

    LOG << "GRAPH: " << graph << endl;
}

void TaskManager::updateCells() {
    for (auto& edge : graph.getEdges()) {
        auto& cell = edge.getCell();
        if (!cell.isFinished())
            cell.update();
        else if (edge.getEnd() == -1) {
            auto& limits = cell.getLimits();
            if (cell.isForwardConvex()) {
                LOG << "End cell with ForwardConvex CP!\n";

                LOG << "Search for already finished edges!\n";
                auto cellLimits = cell.getLimits();
                for (auto& otherEdge : graph.getEdges()) {
                    if (&otherEdge == &edge) // Compare adresses to check if same edge
                        continue;

                    LOG << "Cell " << (void*)(&otherEdge.getCell())
                        << " [" << boolalpha << otherEdge.getCell().isFinished() << ", "
                        << boolalpha << otherEdge.getCell().isForwardConvex() << "]\n";

                    auto otherCellLimits = otherEdge.getCell().getLimits();

                    if (otherEdge.getCell().isFinished()) {
                        if (otherEdge.getCell().isForwardConvex()) {
                            LOG << "I found cell " << (void*)(&otherEdge.getCell()) << " which has reverse CP!\n";
                            LOG << "Test limits: [" << cellLimits << "], [" << otherCellLimits << "]\n";
                            if (isXBoundaryOverlapping(cellLimits, otherCellLimits) ||
                                isXBoundaryOverlapping(otherCellLimits, cellLimits)) {
                                LOG << "Wow, it is overlapping with current cell! 0_o \n";
                                auto yDist = otherEdge.getCell().getEnd().GetY() - otherEdge.getCell().getEnd().GetY();
                                LOG << "Check yDist = " << yDist << "\n";
                                if (fabs(yDist) < ROBOT_CLEARANCE_RADIUS) {
                                    LOG << "Great! That's the same shit!\n";
                                    edge.setEnd(otherEdge.getEnd());
                                    edge.getCell().finish(otherEdge.getCell().getEnd());
                                }
                            }
                        }
                    }
                    else {
                        CVector2 higherCorner;
                        CVector2 lowerCorner;
                        if (otherCellLimits.GetMax().GetX() > cellLimits.GetMax().GetX()) {
                            higherCorner.Set(otherCellLimits.GetMin().GetX(), otherCellLimits.GetMin().GetY());
                            lowerCorner.Set(cellLimits.GetMax().GetX(), cellLimits.GetMin().GetY());
                        }
                        else {
                            higherCorner.Set(cellLimits.GetMin().GetX(), cellLimits.GetMin().GetY());
                            lowerCorner.Set(otherCellLimits.GetMax().GetX(), otherCellLimits.GetMin().GetY());
                        }

                        LOG << "Compare corners [" << higherCorner << "], [" << lowerCorner << "] = "
                            << (higherCorner - lowerCorner).SquareLength() << "\n";
                        if ((higherCorner - lowerCorner).SquareLength() < ROBOT_CLEARANCE) {
                            LOG << "Great! That's the same shit!\n";
                            edge.setEnd(otherEdge.getEnd());
                            edge.getCell().finish(otherEdge.getCell().getEnd());
                        }
                    }
                }

                if (edge.getEnd() == ReebEdge::NO_NODE) {
                    auto newNode = graph.addNode();
                    edge.setEnd(newNode);

                    Real x = 0;
                    auto y = limits.GetMin().GetY();
                    if (fabs(limits.GetMin().GetX()) < fabs(limits.GetMax().GetX()))
                        x = limits.GetMin().GetX() + ROBOT_CLEARANCE_RADIUS;
                    else
                        x = limits.GetMax().GetX() - ROBOT_CLEARANCE_RADIUS;

                    addNewCell(CVector2(x, y), newNode);
                }
            }
            else if (cell.isReverseConvex()) {
                LOG << "End cell with ReverseConvex CP!\n";
                CVector2 beginningLeft(limits.GetMax().GetX() - ROBOT_CLEARANCE_RADIUS, limits.GetMin().GetY());
                CVector2 beginningRight(limits.GetMin().GetX() + ROBOT_CLEARANCE_RADIUS, limits.GetMin().GetY());

                auto newNode = graph.addNode();
                edge.setEnd(newNode);
                addNewCell(beginningLeft, newNode);
                addNewCell(beginningRight, newNode);
            }
        }
    }
}

bool TaskManager::isXBoundaryOverlapping(const CRange<CVector2>& lowerCell, const CRange<CVector2>& upperCell) const {
    return upperCell.GetMax().GetX() > lowerCell.GetMax().GetX() &&
           upperCell.GetMin().GetX() < lowerCell.GetMax().GetX();
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
    if (graph.getCellsSize() != 0)
        THROW_ARGOSEXCEPTION("During initialization none cell should be available!");

    auto unassignedHandlers = getIdleWaitingHandlers();
    CVector2 start = limits.GetMax();
    Real minX = limits.GetMin().GetX();

    LOG << "Initialize TaskManager: [start " << start << "]";
    Task task = {
        CVector2(start.GetX() + ROBOT_CLEARANCE, start.GetY()), start,
        Task::Behavior::Sweep, Task::Status::MoveToBegin
    };
    while (unassignedHandlers.size() > 0) {
        task.begin.SetX( task.begin.GetX() - ROBOT_CLEARANCE);
        if (task.begin.GetX() < minX) {
            task.begin.SetX(start.GetX());
            task.begin.SetY( task.begin.GetY() - ROBOT_CLEARANCE );
            initialLineWidth += ROBOT_CLEARANCE;
        }
        LOG << ", [assign " << to_string(task) << "]";
        auto handler = getClosestHandler(unassignedHandlers, task);
        handler->get().update(task);
        unassignedHandlers.erase(handler);
    }

    if (getIdleWaitingHandlers().size() > 0)
        THROW_ARGOSEXCEPTION("All robots should have assigned start point!");

    unsigned handlersAtStart = 0;
    for (auto handler : handlers)
        if (isNearGoal(handler.get(), handler.get().getCurrentTask().begin)) {
            auto handlerTask = handler.get().getCurrentTask();
            handlerTask.status = Task::Status::Wait;
            handler.get().update(handlerTask);
            handlersAtStart++;
        }
    LOG << ", [" << handlersAtStart << " handlers initizalized]" << endl;

    if (handlers.size() == handlersAtStart) {
        finishWaitingTasks();
        auto newNode = graph.addNode();
        addNewCell(CVector2(0, limits.GetMax().GetY() - initialLineWidth - ROBOT_CLEARANCE), newNode);
        ready = true;
    }
}

void TaskManager::updateMovingHandlers() {
    for (auto handler : handlers) {
        auto handlerTask = handler.get().getCurrentTask();
        if (handlerTask.behavior == Task::Behavior::Sweep)
            updateSweeperTask(handler);
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

TaskManager::HandlersList TaskManager::getIdleWaitingHandlers() const {
    HandlersList unassignedHandlers;
    for(auto handler : handlers)
        if (handler.get().getCurrentTask().behavior == Task::Behavior::Idle &&
            handler.get().getCurrentTask().status == Task::Status::Wait)
            unassignedHandlers.emplace_back(handler);
    return unassignedHandlers;
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