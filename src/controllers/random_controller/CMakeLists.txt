project(random_controller)

add_library(${PROJECT_NAME} MODULE Mbfo.h RandomMovement.cpp)
target_link_libraries(${PROJECT_NAME}
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
