#.rst:
# FindFMILibrary
# -----------
#
# Find the FMI Library.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets if
# FMILibrary has been found::
#
#   FMILibrary::FMILibrary
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module defines the following variables::
#
#   FMILibrary_FOUND           - System has FMILibrary
#   FMILibrary_INCLUDE_DIRS         - Include directories for FMILibrary
#   FMILibrary_LIBRARIES            - imported targets to link against FMILibrary
#
# Readed enviromental variables
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#
# This module reads hints about search locations from variables::
#
#   FMILibrary_ROOT                 - Directory containing the include and lib directories

#=============================================================================
# Copyright 2017 Silvio Traversaro
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.


find_path(FMILibrary_INCLUDE_DIR fmilib.h
          HINTS $ENV{FMILibrary_ROOT}/include)

find_library(FMILibrary_LIBRARY
             NAMES fmilib_shared libfmilib_shared
             HINTS $ENV{FMILibrary_ROOT}/lib)

mark_as_advanced(FMILibrary_INCLUDE_DIR
                 FMILibrary_LIBRARY)

if(FMILibrary_LIBRARY AND FMILibrary_INCLUDE_DIR AND NOT TARGET FMILibrary::FMILibrary)
  add_library(FMILibrary::FMILibrary UNKNOWN IMPORTED)
  set_target_properties(FMILibrary::FMILibrary PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${FMILibrary_INCLUDE_DIR}"
            IMPORTED_LOCATION "${FMILibrary_LIBRARY}")

  set(FMILibrary_LIBRARIES FMILibrary::FMILibrary)
  set(FMILibrary_INCLUDE_DIRS "${FMILibrary_INCLUDE_DIR}")

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(FMILibrary
                                    FOUND_VAR FMILibrary_FOUND
                                    REQUIRED_VARS FMILibrary_LIBRARIES FMILibrary_INCLUDE_DIRS)

  # Set package properties if FeatureSummary was included
  if(COMMAND set_package_properties)
    set_package_properties(FMILibrary PROPERTIES DESCRIPTION "FMILibrary"
                                            URL "http://www.jmodelica.org/FMILibrary")
  endif()
endif()

