cmake_minimum_required(VERSION 3.2)
project(loop_functions)

include(ARGoSCheckQTOpenGL)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
        ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/loop_functions)
include_directories(${CMAKE_SOURCE_DIR} ..)
set(SRC
    RobotsIdsDrawer.cpp
    VoronoiCalculator.cpp
    VoronoiDrawer.cpp
    CoverageGridDrawer.cpp
    CoverageCalculator.cpp
    MbfoLoopFunction.cpp MbfoDrawer.cpp MbfoDrawer.h)

# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_COMPILE_QTOPENGL)
    add_library(${PROJECT_NAME} MODULE ${SRC})

    target_link_libraries(${PROJECT_NAME}
        coverage_search_utils
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_qtopengl
        ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif(ARGOS_COMPILE_QTOPENGL)
