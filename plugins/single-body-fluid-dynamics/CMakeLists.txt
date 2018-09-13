# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
# 
# Licensed under either the GNU Lesser General Public License v3.0 : 
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

add_library(FMISingleBodyFluidDynamicsPlugin SHARED FMISingleBodyFluidDynamicsPlugin.hh FMISingleBodyFluidDynamicsPlugin.cc)
target_include_directories(FMISingleBodyFluidDynamicsPlugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(FMISingleBodyFluidDynamicsPlugin PUBLIC ${GAZEBO_LIBRARIES})
target_link_libraries(FMISingleBodyFluidDynamicsPlugin PUBLIC gazebo_fmi::GazeboFMIPrivateUtils)
target_link_libraries(FMISingleBodyFluidDynamicsPlugin PRIVATE FMILibrary::FMILibrary)
if(NOT MSVC)
    target_link_libraries(FMISingleBodyFluidDynamicsPlugin PUBLIC stdc++fs)
endif()
install(TARGETS FMISingleBodyFluidDynamicsPlugin
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}")
set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS FMISingleBodyFluidDynamicsPlugin)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()
