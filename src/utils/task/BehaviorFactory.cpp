#include "BehaviorFactory.h"

#include "utils/task/behaviors/IdleBehavior.h"
#include "utils/task/behaviors/LeftExplorerBehavior.h"
#include "utils/task/behaviors/RightExplorerBehavior.h"
#include "utils/task/behaviors/SweeperBehavior.h"

using namespace std;
using namespace argos;

shared_ptr<ControllerBehavior> BehaviorFactory::create(Task t) {
    switch (t.behavior) {
        case Task::Behavior::FollowLeftBoundary:
            return make_shared<LeftExplorerBehavior>(sensors, actuators);
        case Task::Behavior::FollowRightBoundary:
            return make_shared<RightExplorerBehavior>(sensors, actuators);
        case Task::Behavior::Sweep:
            return make_shared<SweeperBehavior>(sensors, actuators);
    }

    return make_shared<IdleBehavior>(sensors, actuators);
}