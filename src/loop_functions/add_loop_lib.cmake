cmake_minimum_required(VERSION 3.2)
include(CMakeParseArguments)

function(add_loop_lib NAME)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs SRC DEPENDS)
    cmake_parse_arguments(LOOP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

    add_library(${NAME} SHARED ${LOOP_SRC})
    target_link_libraries(${NAME}
            argos3core_simulator
            argos3plugin_simulator_footbot
            argos3plugin_simulator_entities
            ${LOOP_DEPENDS})
endfunction(add_loop_lib)