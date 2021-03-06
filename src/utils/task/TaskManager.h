#pragma once

#include "ReebGraph.h"
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/range.h>
#include <queue>

class TaskManager {
public:
    TaskManager() = default;
    void init(argos::CRange<argos::CVector2> limits);
    void registerHandler(TaskHandler* handler);
    void unregisterHandler(TaskHandler* handler);
    void assignTasks();

    const auto getCells() const { return graph.getCells(); }
private:
    using HandlersVector = std::vector<TaskHandler*>;
    using HandlersList = std::list<TaskHandler*>;

    HandlersVector handlers;
    ReebGraph graph;
    std::queue<std::pair<Task, std::size_t>> availableTasks;
    argos::CRange<argos::CVector2> limits;
    argos::Real initialLineWidth = 0;
    bool ready = false;


    void initialize();
    void addNewCell(argos::CVector2 beginning, int startNode);

    void finishWaitingTasks();

    void updateCells();
    void updateMovingHandlers();
    void updateSweeperTask(TaskHandler& handler);

    bool isNearGoal(TaskHandler& handler, const argos::CVector2& goal) const;
    HandlersList getIdleWaitingHandlers() const;
    HandlersList::const_iterator getClosestHandler(const HandlersList& handlers, const Task& task) const;

    bool isXBoundaryOverlapping(const argos::CRange<argos::CVector2>& lowerCell,
                                const argos::CRange<argos::CVector2>& upperCell) const;

    void addCellFromForwardConvexCP(const argos::CRange<argos::CVector2>& cellLimits, int endNode);
};


