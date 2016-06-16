project(pso)

add_library(${PROJECT_NAME} MODULE PsoController.h PsoController.cpp)
target_link_libraries(${PROJECT_NAME}
    pso_loop_function
    argos3core_simulator
    argos3plugin_simulator_epuck
    argos3plugin_simulator_genericrobot)
