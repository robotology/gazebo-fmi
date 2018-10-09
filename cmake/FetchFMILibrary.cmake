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
        # FMILibrary shadows the CMAKE_INSTALL_PREFIX with the following line 
        # SET(CMAKE_INSTALL_PREFIX ${FMILIB_INSTALL_PREFIX} CACHE INTERNAL "Prefix prepended to install directories" FORCE)
        # in https://github.com/svn2github/FMILibrary/blob/d49ed3ff2dabc6e17cc4a0c6f3fa6d2ae64a1683/CMakeLists.txt#L87
        # for this reason, we initialize FMILIB_INSTALL_PREFIX to CMAKE_INSTALL_PREFIX to make sure that FMILibrary is installed
        # in the usual location
        set(FMILIB_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE PATH "Prefix prepended to install directories" FORCE)
        # The FMILIB_GENERATE_DOXYGEN_DOC option tries to modifies the CMAKE_INSTALL_PREFIX outside of the install target, 
        # that is a big problem, especially when the default CMAKE_INSTALL_PREFIX is /usr/local, as the configuration will
        # fail unless it is run by root. For this reason we just disable this option 
        option (FMILIB_GENERATE_DOXYGEN_DOC "Generate doxygen doc target" OFF)
        add_subdirectory(${fmilibrary_SOURCE_DIR} ${fmilibrary_BINARY_DIR})
        # We also re-set CMAKE_INSTALL_PREFIX to get rid of the fact that CMAKE_INSTALL_PREFIX is 
        # marked as "INTERNAL"
        set(CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE PATH "Prefix prepended to install directories" FORCE)
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

