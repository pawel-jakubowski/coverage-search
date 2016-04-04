cmake_minimum_required(VERSION 3.2)
project(coverage_search)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(ARGOS3_DIR ${CMAKE_CURRENT_LIST_DIR}/argos3/src)
set(ARGOS3_CMAKE_DIR ${ARGOS3_DIR}/cmake)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS3_CMAKE_DIR})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(argos3)
add_subdirectory(src)
add_subdirectory(src/configurations EXCLUDE_FROM_ALL)