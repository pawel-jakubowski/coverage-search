cmake_minimum_required(VERSION 3.2)
project(configurations)

# Copy configuration files to output directory
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/random.argos
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/random.argos
        COPYONLY)

# Prepare environment variables for targets
set(ENV_VARS
    HOME=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    ARGOS_PLUGIN_PATH=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
set(ENV_CMD ${CMAKE_COMMAND} -E env "${ENV_VARS}")

add_custom_target(experiment_random
        COMMAND ${ENV_CMD} ./argos3 --config-file random.argos
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        DEPENDS random_controller argos3)