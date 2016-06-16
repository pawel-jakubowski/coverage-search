#pragma once

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/range.h>
#include <list>
#include "TaskHandler.h"

class TaskManager {
public:
    void setArenaLimits(argos::CRange<argos::CVector2> limits) { this->limits = limits; }
    void registerHandler(TaskHandler& handler);
    void assignTasks();
private:
    argos::CRange<argos::CVector2> limits;
    std::vector<std::reference_wrapper<TaskHandler>> handlers;

    void finishWaitingTasks() const;
    void setStatusIfNearGoal(TaskHandler& handler, const Task::Status& status, const argos::CVector2& goal) const;
    std::list<std::reference_wrapper<TaskHandler>> getIdleHandlers() const;

    void updateMovingHandlers() const;
};


