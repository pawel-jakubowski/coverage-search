cmake_minimum_required(VERSION 3.2)
project(coverage_loop_function)

add_loop_lib(${PROJECT_NAME} SRC CoverageCalculator.cpp DEPENDS coverage_utils)
add_qt_loop_lib(${PROJECT_NAME}_qt SRC CoverageGridDrawer.cpp DEPENDS ${PROJECT_NAME})