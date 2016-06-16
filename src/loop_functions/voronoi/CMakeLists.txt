cmake_minimum_required(VERSION 3.2)
project(voronoi)

add_loop_lib(${PROJECT_NAME} SRC VoronoiCalculator.cpp DEPENDS voronoi_utils)
add_qt_loop_lib(${PROJECT_NAME}_qt SRC VoronoiDrawer.cpp DEPENDS ${PROJECT_NAME})