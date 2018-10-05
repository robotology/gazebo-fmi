# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

set(GazeboFMIPrivateUtils_HDR
    include/gazebo_fmi/FMUCoSimulation.hh
    include/gazebo_fmi/SDFConfigurationParsing.hh
)

add_library(GazeboFMIPrivateUtils SHARED ${GazeboFMIPrivateUtils_HDR}
                                         FMUCoSimulation.cc
                                         SDFConfigurationParsing.cc)
add_library(gazebo_fmi::GazeboFMIPrivateUtils ALIAS GazeboFMIPrivateUtils)

target_include_directories(GazeboFMIPrivateUtils PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(GazeboFMIPrivateUtils SYSTEM PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(GazeboFMIPrivateUtils PUBLIC ${GAZEBO_LIBRARIES})
target_link_libraries(GazeboFMIPrivateUtils PRIVATE FMILibrary::FMILibrary)

if(NOT MSVC)
    target_link_libraries(GazeboFMIPrivateUtils PUBLIC stdc++fs)
endif()

install(TARGETS GazeboFMIPrivateUtils
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}")


set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS GazeboFMIPrivateUtils)



