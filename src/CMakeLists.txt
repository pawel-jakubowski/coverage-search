cmake_minimum_required(VERSION 3.2)
project(coverage_search_src)

find_package(Lua52 REQUIRED)

# Profiling
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions("-pg")
endif()

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

include_directories(
        .
        ..
        ${ARGOS3_DIR}
        ${LUA_INCLUDE_DIR}
        ${CMAKE_BINARY_DIR})

add_subdirectory(controllers)
add_subdirectory(loop_functions)
add_subdirectory(robots)
add_subdirectory(utils)
add_subdirectory(configurations EXCLUDE_FROM_ALL)

set(ARGOS_SRC_DIR ${ARGOS3_DIR})
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/ARGoS.conf
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/.config/Iridia-ULB/ARGoS.conf)
