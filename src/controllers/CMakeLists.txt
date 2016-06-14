project(controllers)

# Set output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/controllers)

add_subdirectory(mbfo_controller)
add_subdirectory(random_controller)
add_subdirectory(target_controller)
add_subdirectory(pso)
add_subdirectory(cellular_decomposition EXCLUDE_FROM_ALL)
