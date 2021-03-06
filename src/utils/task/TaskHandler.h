#pragma once

#include <argos3/core/utility/math/vector2.h>
#include "Task.h"

#include <mutex>

class TaskHandler {
public:
    virtual ~TaskHandler() = default;

    virtual void update(Task newTask) {
        std::lock_guard<std::mutex> guard(taskUpdateMutex);
        currentTask = newTask;
    }
    virtual Task getCurrentTask() {
        std::lock_guard<std::mutex> guard(taskUpdateMutex);
        return currentTask;
    }
    virtual argos::CVector2 getPosition() const = 0;
    virtual bool isCriticalPoint() const = 0;
    virtual bool isForwardConvexCP() const = 0;
    virtual bool isConcaveCP() const = 0;
    virtual bool isReadyToProceed() const = 0;

protected:
    std::mutex taskUpdateMutex;
    Task currentTask = {argos::CVector2(0,0), argos::CVector2(0,0), Task::Behavior::Idle, Task::Status::Wait};
};
