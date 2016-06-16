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
    } behavior;

    enum class Status {
        Wait = 0,
        MoveToBegin,
        MoveToEnd
    } status;
};
