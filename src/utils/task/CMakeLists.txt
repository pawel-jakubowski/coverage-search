cmake_minimum_required(VERSION 3.2)
project(task_utils)

add_library(${PROJECT_NAME}
        TaskManager.cpp
        TaskCell.cpp
        behaviors/ControllerBehavior.cpp
        behaviors/ExplorerBehavior.cpp
        behaviors/LeftExplorerBehavior.cpp
        behaviors/RightExplorerBehavior.cpp
        BehaviorFactory.cpp
        behaviors/SweeperBehavior.cpp
        Task.cpp
        behaviors/AvoidObstacleBehavior.cpp
        ReebGraph.cpp
        ReebEdge.cpp)
