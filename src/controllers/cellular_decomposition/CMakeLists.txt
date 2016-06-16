project(cellular_decomposition_controller)

add_library(${PROJECT_NAME} MODULE Cellular.h Cellular.cpp)
target_link_libraries(${PROJECT_NAME}
    cellular_loop_function
    argos3core_simulator
    argos3plugin_simulator_footbot
    argos3plugin_simulator_genericrobot)
