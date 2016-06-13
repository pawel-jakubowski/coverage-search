cmake_minimum_required(VERSION 3.2)
project(configurations)

# Prepare environment variables for targets
set(ENV_VARS
    HOME=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    ARGOS_PLUGIN_PATH=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
set(ENV_CMD ${CMAKE_COMMAND} -E env "${ENV_VARS}")

set(ARGOS_EXPERIMENT_LENGTH 1000)
set(ARGOS_TICKS_PER_SEC 10)
#set(ARGOS_RANDOM_SEED "random_seed=\"123\"")

add_subdirectory(random)
add_subdirectory(random_with_ids)
add_subdirectory(random_with_voronoi)
add_subdirectory(random_with_grid)

add_subdirectory(mbfo)
add_subdirectory(dynamic_mbfo)

add_subdirectory(pso)