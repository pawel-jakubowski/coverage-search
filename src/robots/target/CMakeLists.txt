cmake_minimum_required(VERSION 3.2)
project(target_robot)

set(SRC
    dynamics2d_target_model.cpp
    target_entity.cpp
    qtopengl_target.h
    qtopengl_target.cpp
    target_details.h)

set(CMAKE_CURRENT_BINARY_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
add_library(${PROJECT_NAME} SHARED ${SRC})
target_link_libraries(${PROJECT_NAME}
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_qtopengl)

