#pragma once

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/range.h>
#include <list>
#include "TaskCell.h"

class TaskManager {
public:
    TaskManager() = default;
    void init(argos::CRange<argos::CVector2> limits);
    void registerHandler(TaskHandler& handler);
    void unregisterHandler(TaskHandler& handler);
    void assignTasks();

    const auto getCells() const { return cells; }
private:
    using HandlersVector = std::vector<std::reference_wrapper<TaskHandler>>;
    using HandlersList = std::list<std::reference_wrapper<TaskHandler>>;

    HandlersVector handlers;
    std::vector<TaskCell> cells;
    std::list<Task> availableTasks;
    argos::CRange<argos::CVector2> limits;
    argos::Real initialLineWidth = 0;
    bool ready = false;


    void initialize();
    void addNewCell(argos::CRange<argos::CVector2> limits);

    void finishWaitingTasks();
    void setStatusIfNearGoal(TaskHandler& handler, const Task::Status& status, const argos::CVector2& goal);

    void updateMovingHandlers();
    void updateExplorerTask(TaskHandler& handler);
    void updateSweeperTask(TaskHandler& handler);

    bool isNearGoal(TaskHandler& handler, const argos::CVector2& goal) const;
    HandlersList getIdleHandlers() const;
    HandlersList::const_iterator getClosestHandler(const HandlersList& handlers, const Task& task) const;
};


