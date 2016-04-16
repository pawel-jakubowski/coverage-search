cmake_minimum_required(VERSION 3.2)
project(coverage_search_src)

find_package(Lua52 REQUIRED)

include_directories(
        .
        ..
        ${ARGOS3_DIR}
        ${LUA_INCLUDE_DIR}
        ${CMAKE_BINARY_DIR})

add_subdirectory(controllers)
add_subdirectory(loop_functions)
add_subdirectory(utils)
add_subdirectory(configurations EXCLUDE_FROM_ALL)