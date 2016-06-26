#include "BehaviorFactory.h"

#include "IdleBehavior.h"
#include "LeftExplorerBehavior.h"
#include "RightExplorerBehavior.h"
#include "SweeperBehavior.h"

using namespace std;
using namespace argos;

shared_ptr<ControllerBehavior> BehaviorFactory::create(Task t) {
    switch (t.behavior) {
        case Task::Behavior::FollowLeftBoundary:
//            LOG << "BehaviorFactory: Create left explorer behavior!" << endl;
            return make_shared<LeftExplorerBehavior>(sensors, actuators);
        case Task::Behavior::FollowRightBoundary:
//            LOG << "BehaviorFactory: Create right explorer behavior!" << endl;
            return make_shared<RightExplorerBehavior>(sensors, actuators);
        case Task::Behavior::Sweep:
//            LOG << "BehaviorFactory: Create sweeper behavior!" << endl;
            return make_shared<SweeperBehavior>(sensors, actuators);
    }

//    LOG << "BehaviorFactory: Create idle behavior!" << endl;
    return make_shared<IdleBehavior>(sensors, actuators);
}