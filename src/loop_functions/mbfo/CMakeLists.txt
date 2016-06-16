cmake_minimum_required(VERSION 3.2)
project(mbfo_loop_function)

add_loop_lib(${PROJECT_NAME} SRC MbfoLoopFunction.cpp DynamicMbfoLoopFunction.cpp
        DEPENDS coverage_utils voronoi_utils)
add_qt_loop_lib(${PROJECT_NAME}_qt SRC MbfoDrawer.cpp DEPENDS ${PROJECT_NAME})