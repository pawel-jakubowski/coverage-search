cmake_minimum_required(VERSION 3.2)
project(dynamic_mbfo)

set(CONFIG_FILE ${PROJECT_NAME}.argos)
set(ARGOS_TEXTURES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../textures)

#set(ARGOS_ROBOTS_NUMBER 50)
#set(ARGOS_AREA_HALF_SIDE_IN_M 5)

set(ARGOS_ROBOTS_NUMBER 3)
set(ARGOS_AREA_HALF_SIDE_IN_M 1)

set(ARGOS_WALL_THICKNESS_IN_M 0.01)
math(EXPR ARGOS_AREA_SIDE_IN_M ${ARGOS_AREA_HALF_SIDE_IN_M}*2)
math(EXPR ARGOS_CAMERA_1 ${ARGOS_AREA_SIDE_IN_M}*2)

# Copy configuration files to output directory
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_FILE}
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}/${CONFIG_FILE})

add_custom_target(experiment_${PROJECT_NAME}
        COMMAND ${ENV_CMD} ./argos3 --config-file ${PROJECT_NAME}/${CONFIG_FILE} ${ARGOS_OPTIONS}
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        DEPENDS mbfo_controller loop_functions argos3)

#Profiling
add_custom_target(profile_${PROJECT_NAME}
        COMMAND valgrind --tool=callgrind ./argos3 --config-file ${PROJECT_NAME}/${CONFIG_FILE} ${ARGOS_OPTIONS}
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        DEPENDS mbfo_controller loop_functions argos3)