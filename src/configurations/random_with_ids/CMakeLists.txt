cmake_minimum_required(VERSION 3.2)
project(random_with_ids)

set(CONFIG_FILE random_with_ids.argos)

# Copy configuration files to output directory
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_FILE}
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}/${CONFIG_FILE}
        COPYONLY)

add_custom_target(experiment_${PROJECT_NAME}
        COMMAND ${ENV_CMD} ./argos3 --config-file ${PROJECT_NAME}/${CONFIG_FILE}
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        DEPENDS random_controller robots_ids_loop_function_qt argos3)