# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
# 
# Licensed under either the GNU Lesser General Public License v3.0 : 
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

add_library(GazeboFMIUtils INTERFACE)
target_include_directories(GazeboFMIUtils INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
                                                    ${GAZEBO_INCLUDE_DIRS})

add_library(FMIActuatorPlugin SHARED FMIActuatorPlugin.hh FMIActuatorPlugin.cc
                                     FMUCoSimulation.hh FMUCoSimulation.cc)
target_include_directories(FMIActuatorPlugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(FMIActuatorPlugin PUBLIC ${GAZEBO_LIBRARIES} FMILibrary::FMILibrary
                                        PRIVATE GazeboFMIUtils)
if(NOT MSVC)
    target_link_libraries(FMIActuatorPlugin PUBLIC stdc++fs)
endif()
install(TARGETS FMIActuatorPlugin 
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}")
set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS FMIActuatorPlugin)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

