cmake_minimum_required(VERSION 3.2)
project(coverage_search_utils)

add_definitions("-fPIC")

add_subdirectory(coverage)
add_subdirectory(math)
add_subdirectory(task)
add_subdirectory(voronoi)
