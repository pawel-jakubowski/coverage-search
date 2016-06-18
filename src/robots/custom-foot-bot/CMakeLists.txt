#
# Foot-bot headers
#

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_FOOTBOT_SIMULATOR
  simulator/colored_blob_perspective_camera_default_sensor.h
  simulator/dynamics2d_footbot_model.h
  simulator/footbot_entity.h)

#
# Custom Foot-bot sources
#

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_FOOTBOT
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_FOOTBOT}
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_FOOTBOT_SIMULATOR}
  simulator/colored_blob_perspective_camera_default_sensor.cpp
  simulator/dynamics2d_footbot_model.cpp
  simulator/footbot_entity.cpp)

# Compile the graphical visualization only if the necessary libraries have been found
include(ARGoSCheckQTOpenGL)
if(ARGOS_COMPILE_QTOPENGL)
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_FOOTBOT_SIMULATOR
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_FOOTBOT_SIMULATOR}
  simulator/qtopengl_footbot.h)
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_FOOTBOT
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_FOOTBOT}
  simulator/qtopengl_footbot.h
  simulator/qtopengl_footbot.cpp)
endif(ARGOS_COMPILE_QTOPENGL)

#
# Create foot-bot plugin
#
add_library(argos3plugin_simulator_custom_footbot SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_FOOTBOT})
target_link_libraries(argos3plugin_simulator_custom_footbot
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_dynamics2d)

if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_simulator_custom_footbot argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_FOOTBOT_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/foot-bot/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_FOOTBOT_SIMULATOR}      DESTINATION include/argos3/plugins/robots/foot-bot/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_simulator_custom_footbot
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
