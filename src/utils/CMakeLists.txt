cmake_minimum_required(VERSION 3.2)
project(coverage_search_utils)

set(SRC
        VoronoiDiagram.cpp
        liang-barsky.cpp
        CoverageGrid.cpp)

add_library(${PROJECT_NAME} SHARED ${SRC})
