#pragma once

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/range.h>
#include <list>
#include "TaskCell.h"

class TaskManager {
public:
    TaskManager() = default;
    void addNewCell(argos::CRange<argos::CVector2> limits);
    void registerHandler(TaskHandler& handler);
    void unregisterHandler(TaskHandler& handler);
    void assignTasks();

    const auto getCells() const { return cells; }
private:
    std::vector<TaskCell> cells;
    std::vector<std::reference_wrapper<TaskHandler>> handlers;
    std::list<Task> availableTasks;
    bool ready = false;

    std::list<std::reference_wrapper<TaskHandler>> getIdleHandlers() const;

    void initialize();

    void finishWaitingTasks();
    void setStatusIfNearGoal(TaskHandler& handler, const Task::Status& status, const argos::CVector2& goal);

    void updateMovingHandlers();
    void updateExplorerTask(TaskHandler& handler);
    void updateSweeperTask(TaskHandler& handler);

    bool isNearGoal(TaskHandler& handler, const argos::CVector2& goal) const;
};


