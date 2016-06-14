#pragma once

enum class Behavior {
    Sweep = 0,
    MoveLeft,
    MoveRight,
    FollowLeftBoundary,
    FollowRightBoundary
};

enum class TaskType {
    Sweep = 0,
    Cycle,
    WaitForNewTask,
    MoveToNextTask,
    WaitForTaskToBegin
};