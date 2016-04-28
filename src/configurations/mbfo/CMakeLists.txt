cmake_minimum_required(VERSION 3.2)
project(mbfo)

set(CONFIG_FILE mbfo.argos)

# Copy configuration files to output directory
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_FILE}
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}/${CONFIG_FILE}
        COPYONLY)

add_custom_target(experiment_${PROJECT_NAME}
        COMMAND ${ENV_CMD} ./argos3 --config-file ${PROJECT_NAME}/${CONFIG_FILE} ${ARGOS_OPTIONS}
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        DEPENDS mbfo_controller loop_functions argos3)