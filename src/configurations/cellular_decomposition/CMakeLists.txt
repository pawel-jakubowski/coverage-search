cmake_minimum_required(VERSION 3.2)
project(cellular_decomposition)

set(CONFIG_FILE cellular_decomposition.argos)
set(ARGOS_TEXTURES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../textures)

if(NOT ARGOS_ROBOTS_NUMBER)
    set(ARGOS_ROBOTS_NUMBER 5)
endif()
if(NOT ARGOS_TARGETS_NUMBER)
    set(ARGOS_TARGETS_NUMBER 5)
endif()
if(NOT ARGOS_AREA_HALF_SIDE_IN_M)
    set(ARGOS_AREA_HALF_SIDE_IN_M 3)
endif()
if(NOT ARGOS_AREA_SIDE_IN_M)
    math(EXPR ARGOS_AREA_SIDE_IN_M ${ARGOS_AREA_HALF_SIDE_IN_M}*2)
endif()
if(NOT ARGOS_CAMERA_1)
    math(EXPR ARGOS_CAMERA_1 ${ARGOS_AREA_SIDE_IN_M}*2)
endif()
if(NOT ARGOS_LOG)
    set(ARGOS_LOG "cellular.log")
endif()

execute_process(
        COMMAND python -c "import sys; sys.stdout.write(str(${ARGOS_AREA_HALF_SIDE_IN_M}-0.4))"
        OUTPUT_VARIABLE ARGOS_ROBOTS_HALF_AREA_SIDE_BEGINNIG_IN_M)
execute_process(
        COMMAND python -c "import sys; sys.stdout.write(str(${ARGOS_AREA_HALF_SIDE_IN_M}-0.8))"
        OUTPUT_VARIABLE ARGOS_TARGET_HALF_AREA_SIDE_BEGINNIG_IN_M)
execute_process(
        COMMAND python -c "import sys; sys.stdout.write(str(${ARGOS_AREA_HALF_SIDE_IN_M}-0.6))"
        OUTPUT_VARIABLE ARGOS_TARGET_HALF_AREA_SIDE_IN_M)
set(ARGOS_WALL_THICKNESS_IN_M 0.02)

# Copy configuration files to output directory
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_FILE}
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}/${CONFIG_FILE})

add_custom_target(experiment_${PROJECT_NAME}
        COMMAND ${ENV_CMD} ./argos3 --config-file ${PROJECT_NAME}/${CONFIG_FILE} ${ARGOS_OPTIONS}
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        DEPENDS
            cellular_decomposition_controller
            cellular_loop_function
            cellular_loop_function_qt
            target_controller
            target_robot
            argos3)
