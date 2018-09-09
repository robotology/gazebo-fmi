#.rst:
# FindMATIO
# -----------
#
# Find the MATLAB MAT file I/O library library.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets if
# MATIO has been found::
#
#   MATIO::MATIO
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module defines the following variables::
#
#   MATIO_FOUND                - System has MATIO
#   MATIO_INCLUDE_DIRS         - Include directories for MATIO
#   MATIO_LIBRARIES            - imported targets to link against MATIO
#
# Readed enviromental variables
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#
# This module reads hints about search locations from variables::
#
#   MATIO_ROOT                 - Directory containing the include and lib directories

# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

find_path(MATIO_INCLUDE_DIR matio.h
          HINTS $ENV{MATIO_ROOT})

find_library(MATIO_LIBRARY
             NAMES libmatio matio
             HINTS $ENV{MATIO_ROOT})

mark_as_advanced(MATIO_INCLUDE_DIR
                 MATIO_LIBRARY)

if( MATIO_LIBRARY AND MATIO_INCLUDE_DIR AND NOT TARGET MATIO::MATIO)
  add_library(MATIO::MATIO UNKNOWN IMPORTED)
  set_target_properties(MATIO::MATIO PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${MATIO_INCLUDE_DIR}"
            IMPORTED_LOCATION "${MATIO_LIBRARY}")

  set(MATIO_LIBRARIES MATIO::MATIO)
  set(MATIO_INCLUDE_DIRS "${MATIO_INCLUDE_DIR}")
endif()


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MATIO
                                  FOUND_VAR MATIO_FOUND
                                  REQUIRED_VARS MATIO_LIBRARIES MATIO_INCLUDE_DIRS)

# Set package properties if FeatureSummary was included
if(COMMAND set_package_properties)
  set_package_properties(MATIO PROPERTIES DESCRIPTION "MATLAB MAT File I/O Library"
                                          URL "http://matio.sf.net")
endif()

