cmake_minimum_required(VERSION 3.2)
project(voronoi_utils)

add_library(${PROJECT_NAME} VoronoiDiagram.cpp VoronoiCell.cpp)
target_link_libraries(${PROJECT_NAME} math_utils)
