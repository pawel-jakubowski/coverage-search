project(mbfo_controller)

add_library(${PROJECT_NAME} MODULE Mbfo.h Mbfo.cpp)
target_link_libraries(${PROJECT_NAME}
    mbfo_loop_function
    argos3core_simulator
    argos3plugin_simulator_footbot
    argos3plugin_simulator_genericrobot)
