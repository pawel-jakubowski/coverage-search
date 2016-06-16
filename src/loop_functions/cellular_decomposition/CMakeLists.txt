cmake_minimum_required(VERSION 3.2)
project(cellular_loop_function)

add_loop_lib(${PROJECT_NAME} SRC CellularDecomposition.cpp
        DEPENDS coverage_utils task_utils)
add_qt_loop_lib(${PROJECT_NAME}_qt SRC CellularDrawer.cpp DEPENDS ${PROJECT_NAME})