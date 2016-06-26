cmake_minimum_required(VERSION 3.2)
project(task_utils)

add_library(${PROJECT_NAME}
        TaskManager.cpp
        TaskCell.cpp
        ControllerBehavior.cpp
        ExplorerBehavior.cpp
        LeftExplorerBehavior.cpp
        RightExplorerBehavior.cpp
        BehaviorFactory.cpp
        SweeperBehavior.cpp Task.cpp)
