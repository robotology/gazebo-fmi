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
                        OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  add_custom_target(generate-fmu-damper ALL DEPENDS Damper.fmu)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/Damper.fmu DESTINATION ${GAZEBO_FMI_EXAMPLES_INSTALL_DIR})
endif()

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/Damper.mo DESTINATION ${GAZEBO_FMI_EXAMPLES_INSTALL_DIR})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/damped_pendulum.world DESTINATION ${GAZEBO_FMI_EXAMPLES_INSTALL_DIR})

