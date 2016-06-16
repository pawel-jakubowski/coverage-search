cmake_minimum_required(VERSION 3.2)
include(./add_loop_lib.cmake)
include(ARGoSCheckQTOpenGL)

function(add_qt_loop_lib NAME)
    if(NOT ARGOS_COMPILE_QTOPENGL)
        message(FATAL_ERROR "No QT libraries was found!")
    endif()

    set(options)
    set(oneValueArgs)
    set(multiValueArgs SRC DEPENDS)
    cmake_parse_arguments(QT_LOOP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

    add_loop_lib(${NAME}
            SRC ${QT_LOOP_SRC}
            DEPENDS ${QT_LOOP_DEPENDS}
            argos3core_simulator ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endfunction(add_qt_loop_lib)