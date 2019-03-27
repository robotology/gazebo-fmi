# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
# 
# Licensed under either the GNU Lesser General Public License v3.0 : 
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

file(GLOB allSrc "*.h" "*.cpp")
add_library(FMIStateLoggerPlugin SHARED ${allSrc})
target_include_directories(FMIStateLoggerPlugin SYSTEM PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(FMIStateLoggerPlugin PUBLIC ${GAZEBO_LIBRARIES})
target_link_libraries(FMIStateLoggerPlugin PUBLIC gazebo_fmi::GazeboFMIPrivateUtils)
target_link_libraries(FMIStateLoggerPlugin PRIVATE FMILibrary::FMILibrary)

install(TARGETS FMIStateLoggerPlugin 
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}")
set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS FMIStateLoggerPlugin)


