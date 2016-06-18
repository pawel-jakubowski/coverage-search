#pragma once

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/range.h>
#include <list>
#include "TaskHandler.h"

class TaskManager {
public:
    TaskManager() = default;
    void addNewCell(argos::CRange<argos::CVector2> limits);
    void registerHandler(TaskHandler& handler);
    void unregisterHandler(TaskHandler& handler);
    void assignTasks();
private:
    std::vector<std::reference_wrapper<TaskHandler>> handlers;
    std::list<Task> availableTasks;

    void finishWaitingTasks() const;
    void setStatusIfNearGoal(TaskHandler& handler, const Task::Status& status, const argos::CVector2& goal) const;
    std::list<std::reference_wrapper<TaskHandler>> getIdleHandlers() const;

    void updateMovingHandlers() const;
};


