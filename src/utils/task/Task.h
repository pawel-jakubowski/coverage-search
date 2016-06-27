#pragma once

#include <argos3/core/utility/math/vector2.h>

struct Task {
    argos::CVector2 begin;
    argos::CVector2 end;

    enum class Behavior {
        Idle = 0,
        Sweep,
        FollowLeftBoundary,
        FollowRightBoundary
    } behavior = Behavior::Idle;

    enum class Status {
        Wait = 0,
        MoveToBegin,
        Prepare,
        Proceed
    } status = Status::Wait;
};

std::string to_string(Task t);
std::string to_string(Task::Behavior b);
std::string to_string(Task::Status s);
