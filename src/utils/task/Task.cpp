#include "Task.h"
#include <sstream>
#include <iomanip>

using namespace std;

string to_string(Task t) {
    stringstream s;
    s   << "[{"
    << setprecision(2) << t.begin.GetX() << ", "
    << setprecision(2) << t.begin.GetY()
    << "}, {"
    << setprecision(2) << t.end.GetX() << ", "
    << setprecision(2) << t.end.GetY()
    << "}, "
    << to_string(t.behavior) << ", "
    << to_string(t.status)
    << "]";
    return s.str();
}

string to_string(Task::Behavior b) {
    switch(b) {
        case Task::Behavior::Idle:
            return "IDLE";
        case Task::Behavior::Sweep:
            return "SWEEP";
        case Task::Behavior::FollowLeftBoundary:
            return "FLEFT";
        case Task::Behavior::FollowRightBoundary:
            return "FRIGHT";
    }
    return to_string(static_cast<unsigned>(b));
}

string to_string(Task::Status s) {
    switch(s) {
        case Task::Status::Wait:
            return "WAIT";
        case Task::Status::MoveToBegin:
            return "TO BEGIN";
        case Task::Status::Proceed:
            return "PROCEED";
    }
    return to_string(static_cast<unsigned>(s));
}