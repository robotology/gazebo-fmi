# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

include(FMIUtils)

# Only create FMU if the OpenModelica compiler (omc) is found in the path
find_program(OMC_COMPILER NAMES omc)

if(OMC_COMPILER)
  file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/fmu)
  omc_compile_mo_to_fmu(INPUT_MO ${CMAKE_CURRENT_SOURCE_DIR}/Damper.mo
                        MODEL_NAME Damper
                        OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/fmu)
  add_custom_target(generate-fmu-damper ALL DEPENDS Damper.fmu)
endif()

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/damped_pendulum.world DESTINATION ${PROJECT_BINARY_DIR}/examples)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/setup-build.sh.in ${PROJECT_BINARY_DIR}/setup.sh)
