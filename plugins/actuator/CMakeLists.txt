# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
# 
# Licensed under either the GNU Lesser General Public License v3.0 : 
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

add_library(FMIActuatorPlugin SHARED FMIActuatorPlugin.hh FMIActuatorPlugin.cc)
target_include_directories(FMIActuatorPlugin SYSTEM PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(FMIActuatorPlugin PUBLIC ${GAZEBO_LIBRARIES})
target_link_libraries(FMIActuatorPlugin PUBLIC gazebo_fmi::GazeboFMIPrivateUtils)
target_link_libraries(FMIActuatorPlugin PRIVATE FMILibrary::FMILibrary)

install(TARGETS FMIActuatorPlugin 
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}")
set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS FMIActuatorPlugin)

# Build examples FMUs
add_subdirectory(examples)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

