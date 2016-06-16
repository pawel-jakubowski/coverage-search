#pragma once

#include <argos3/core/utility/math/vector2.h>
#include "Task.h"

class TaskHandler {
public:
    virtual ~TaskHandler() = default;

    virtual void update(Task newTask) { currentTask = newTask; }
    virtual Task& getCurrentTask() { return currentTask; }
    virtual argos::CVector2 getPostion() = 0;

protected:
    Task currentTask = {argos::CVector2(0,0), argos::CVector2(0,0), Task::Behavior::Idle, Task::Status::Wait};
};
