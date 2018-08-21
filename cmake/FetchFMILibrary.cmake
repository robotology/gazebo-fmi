# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

# Fetch FMI library for downloading and installing
include(FetchContent)
FetchContent_Declare(FMILibrary
                     GIT_REPOSITORY https://github.com/svn2github/FMILibrary.git)

FetchContent_GetProperties(FMILibrary)
if(NOT FMILibrary_POPULATED)
        FetchContent_Populate(FMILibrary)
        set(BUILD_SHARED_LIBS OFF)
        option (FMILIB_BUILD_TESTS "Build tests" OFF)
        add_subdirectory(${fmilibrary_SOURCE_DIR} ${fmilibrary_BINARY_DIR})
        set(BUILD_SHARED_LIBS ON)
endif()

add_library(FMILibrary_FMILibrary INTERFACE)
target_include_directories(FMILibrary_FMILibrary INTERFACE ${fmilibrary_SOURCE_DIR}/src/CAPI/include
                                                           ${fmilibrary_SOURCE_DIR}/src/Import/include
                                                           ${fmilibrary_SOURCE_DIR}/src/Util/include
                                                           ${fmilibrary_SOURCE_DIR}/src/XML/include
                                                           ${fmilibrary_SOURCE_DIR}/src/ZIP/include
                                                           ${fmilibrary_SOURCE_DIR}/ThirdParty/FMI/default
                                                           ${fmilibrary_SOURCE_DIR}/Config.cmake
                                                           ${fmilibrary_BINARY_DIR})
target_link_libraries(FMILibrary_FMILibrary INTERFACE fmilib)
add_library(FMILibrary::FMILibrary ALIAS FMILibrary_FMILibrary)

