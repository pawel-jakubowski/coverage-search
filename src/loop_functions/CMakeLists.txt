cmake_minimum_required(VERSION 3.2)
project(loop_functions)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
        ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/loop_functions)

include_directories(${CMAKE_SOURCE_DIR} ..)

include(./add_loop_lib.cmake)
include(./add_qt_loop_lib.cmake)

add_subdirectory(cellular_decomposition)
add_subdirectory(coverage)
add_subdirectory(mbfo)
add_subdirectory(robots_ids)
add_subdirectory(pso)
add_subdirectory(voronoi)
